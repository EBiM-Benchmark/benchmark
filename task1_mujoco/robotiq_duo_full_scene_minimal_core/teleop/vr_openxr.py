"""OpenXR backend for VR teleoperation — no SteamVR/Steam required.

Talks to the system's active OpenXR runtime, so it works across vendors:
  Quest 2/3   -> Meta Quest Link app (Link cable or Air Link)
  Pico        -> Pico Connect streaming
  WMR/Index/Vive -> their own runtimes (SteamVR also works if preferred)

Input uses OpenXR actions with suggested bindings for the major controller
profiles (Touch, Index, Vive, simple), so different controller brands map
automatically. The simulator view is shown inside the headset as a floating
virtual screen (mono), anchored in front of the stage origin.
"""

from __future__ import annotations

import ctypes
import math
import threading
from ctypes import POINTER, byref, cast, pointer

import glfw
import numpy as np
import xr
from OpenGL import GL
from xr.utils.gl import ContextObject
from xr.utils.gl.glfw_util import GLFWOffscreenContextProvider


class HandState:
    __slots__ = (
        "pos",
        "rot",
        "trigger",
        "grip",
        "stick",
        "stick_click",
        "a",
        "b",
        "valid",
    )

    def __init__(self):
        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.trigger = 0.0
        self.grip = 0.0
        self.stick = np.zeros(2)
        self.stick_click = False
        self.a = False
        self.b = False
        self.valid = False


def _quat_xyzw_to_mat(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ]
    )


def _pose_to_mat4(pose) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _quat_xyzw_to_mat(pose.orientation)
    m[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return m


def _projection_from_fov(fov, near: float, far: float) -> np.ndarray:
    left, right = math.tan(fov.angle_left), math.tan(fov.angle_right)
    down, up = math.tan(fov.angle_down), math.tan(fov.angle_up)
    w, h = right - left, up - down
    m = np.zeros((4, 4))
    m[0, 0] = 2.0 / w
    m[0, 2] = (right + left) / w
    m[1, 1] = 2.0 / h
    m[1, 2] = (up + down) / h
    m[2, 2] = -(far + near) / (far - near)
    m[2, 3] = -2.0 * far * near / (far - near)
    m[3, 2] = -1.0
    return m


_VERT = """
#version 330 core
layout(location = 0) in vec2 in_pos;
layout(location = 1) in vec2 in_uv;
uniform mat4 mvp;
out vec2 uv;
void main() {
    gl_Position = mvp * vec4(in_pos, 0.0, 1.0);
    uv = in_uv;
}
"""

_FRAG = """
#version 330 core
in vec2 uv;
uniform sampler2D tex;
out vec4 color;
void main() {
    color = vec4(texture(tex, vec2(uv.x, 1.0 - uv.y)).rgb, 1.0);
}
"""

# where the virtual screen floats in the stage (meters): straight ahead of
# the play-space center at eye-ish height
SCREEN_CENTER = np.array([0.0, 1.4, -1.6])
SCREEN_WIDTH = 2.2


class OpenXRTeleop:
    """OpenXR session + cross-vendor controller actions + in-headset screen."""

    def __init__(self, image_size=(1280, 720)):
        self.image_w, self.image_h = image_size
        self.context: ContextObject | None = None
        self._provider = None
        self.hands = {"left": HandState(), "right": HandState()}
        self._tex = None
        self._prog = None
        self._vao = None
        # threading: the XR frame loop runs at headset rate in its own
        # thread; the control loop exchanges data through these
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = False
        self._pending_frame: np.ndarray | None = None
        self._haptics = {"left": 0.0, "right": 0.0}
        self.error: BaseException | None = None

    # ---------------------------------------------------------------- setup
    @staticmethod
    def _cleanup_failed_context(ctx) -> None:
        """Destroy whatever a half-entered ContextObject created.

        Destroying the instance tears down all child handles; without this a
        retry hits 'Loader does not support simultaneous XrInstances'.
        """
        try:
            instance = getattr(ctx, "instance", None)
            if instance is not None:
                xr.destroy_instance(instance)
        except Exception:
            pass
        for attr in (
            "instance",
            "session",
            "space",
            "default_action_set",
            "graphics",
        ):
            if hasattr(ctx, attr):
                setattr(ctx, attr, None)

    def _make_context(self, ref_space_type) -> ContextObject:
        ctx = ContextObject(
            context_provider=self._provider,
            instance_create_info=xr.InstanceCreateInfo(
                application_info=xr.ApplicationInfo(
                    application_name="duo_vr_teleop",
                    application_version=1,
                    engine_name="mujoco",
                ),
                enabled_extension_names=[xr.KHR_OPENGL_ENABLE_EXTENSION_NAME],
            ),
            reference_space_create_info=xr.ReferenceSpaceCreateInfo(
                reference_space_type=ref_space_type,
            ),
        )
        try:
            ctx.__enter__()
        except BaseException:
            self._cleanup_failed_context(ctx)
            raise
        return ctx

    def __enter__(self) -> OpenXRTeleop:
        if not glfw.init():
            raise RuntimeError("glfw.init failed")
        self._provider = GLFWOffscreenContextProvider()
        ffu_error = getattr(xr, "FormFactorUnavailableError", xr.XrException)
        import time as _time

        deadline = _time.time() + 180.0
        announced = False
        while True:
            try:
                self.context = self._make_context(xr.ReferenceSpaceType.STAGE)
                break
            except ffu_error:
                # runtime is up but sees no active headset: not in Link mode
                # yet, or not on a head. Wait instead of dying so the operator
                # can start the script first and then put the headset on.
                if _time.time() > deadline:
                    raise
                if not announced:
                    print(
                        "[vr_openxr] waiting for the headset... enter Quest Link"
                        " mode in the headset and put it on (Ctrl+C to abort)"
                    )
                    announced = True
                _time.sleep(3.0)
            except BaseException as first_exc:
                print(f"[vr_openxr] STAGE setup failed: {first_exc!r}; retrying with LOCAL space")
                try:
                    self.context = self._make_context(xr.ReferenceSpaceType.LOCAL)
                    break
                except BaseException:
                    # the first error is usually the meaningful one
                    raise first_exc
        self._setup_actions()
        self._setup_gl()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.context is not None:
            self.context.__exit__(exc_type, exc, tb)
            self.context = None
        # NOTE: no glfw.terminate() here — glfw state is shared with the
        # mujoco passive viewer running in the same process

    def _setup_actions(self) -> None:
        ctx = self.context
        inst = ctx.instance
        self.hand_paths = (xr.Path * 2)(
            xr.string_to_path(inst, "/user/hand/left"),
            xr.string_to_path(inst, "/user/hand/right"),
        )

        def make_action(name, kind):
            return xr.create_action(
                action_set=ctx.default_action_set,
                create_info=xr.ActionCreateInfo(
                    action_type=kind,
                    action_name=name,
                    localized_action_name=name.replace("_", " "),
                    count_subaction_paths=2,
                    subaction_paths=self.hand_paths,
                ),
            )

        self.a_pose = make_action("hand_pose", xr.ActionType.POSE_INPUT)
        self.a_trigger = make_action("trigger", xr.ActionType.FLOAT_INPUT)
        self.a_squeeze = make_action("squeeze", xr.ActionType.FLOAT_INPUT)
        self.a_stick = make_action("thumbstick", xr.ActionType.VECTOR2F_INPUT)
        self.a_stick_click = make_action("thumbstick_click", xr.ActionType.BOOLEAN_INPUT)
        self.a_primary = make_action("primary_button", xr.ActionType.BOOLEAN_INPUT)
        self.a_secondary = make_action("secondary_button", xr.ActionType.BOOLEAN_INPUT)
        self.a_haptic = make_action("haptic", xr.ActionType.VIBRATION_OUTPUT)

        def p(path):
            return xr.string_to_path(inst, path)

        profiles = {
            "/interaction_profiles/oculus/touch_controller": [
                (self.a_pose, "/user/hand/left/input/grip/pose"),
                (self.a_pose, "/user/hand/right/input/grip/pose"),
                (self.a_trigger, "/user/hand/left/input/trigger/value"),
                (self.a_trigger, "/user/hand/right/input/trigger/value"),
                (self.a_squeeze, "/user/hand/left/input/squeeze/value"),
                (self.a_squeeze, "/user/hand/right/input/squeeze/value"),
                (self.a_stick, "/user/hand/left/input/thumbstick"),
                (self.a_stick, "/user/hand/right/input/thumbstick"),
                (self.a_stick_click, "/user/hand/left/input/thumbstick/click"),
                (
                    self.a_stick_click,
                    "/user/hand/right/input/thumbstick/click",
                ),
                (self.a_primary, "/user/hand/left/input/x/click"),
                (self.a_primary, "/user/hand/right/input/a/click"),
                (self.a_secondary, "/user/hand/left/input/y/click"),
                (self.a_secondary, "/user/hand/right/input/b/click"),
                (self.a_haptic, "/user/hand/left/output/haptic"),
                (self.a_haptic, "/user/hand/right/output/haptic"),
            ],
            "/interaction_profiles/valve/index_controller": [
                (self.a_pose, "/user/hand/left/input/grip/pose"),
                (self.a_pose, "/user/hand/right/input/grip/pose"),
                (self.a_trigger, "/user/hand/left/input/trigger/value"),
                (self.a_trigger, "/user/hand/right/input/trigger/value"),
                (self.a_squeeze, "/user/hand/left/input/squeeze/value"),
                (self.a_squeeze, "/user/hand/right/input/squeeze/value"),
                (self.a_stick, "/user/hand/left/input/thumbstick"),
                (self.a_stick, "/user/hand/right/input/thumbstick"),
                (self.a_stick_click, "/user/hand/left/input/thumbstick/click"),
                (
                    self.a_stick_click,
                    "/user/hand/right/input/thumbstick/click",
                ),
                (self.a_primary, "/user/hand/left/input/a/click"),
                (self.a_primary, "/user/hand/right/input/a/click"),
                (self.a_secondary, "/user/hand/left/input/b/click"),
                (self.a_secondary, "/user/hand/right/input/b/click"),
                (self.a_haptic, "/user/hand/left/output/haptic"),
                (self.a_haptic, "/user/hand/right/output/haptic"),
            ],
            "/interaction_profiles/htc/vive_controller": [
                (self.a_pose, "/user/hand/left/input/grip/pose"),
                (self.a_pose, "/user/hand/right/input/grip/pose"),
                (self.a_trigger, "/user/hand/left/input/trigger/value"),
                (self.a_trigger, "/user/hand/right/input/trigger/value"),
                (self.a_squeeze, "/user/hand/left/input/squeeze/click"),
                (self.a_squeeze, "/user/hand/right/input/squeeze/click"),
                (self.a_stick, "/user/hand/left/input/trackpad"),
                (self.a_stick, "/user/hand/right/input/trackpad"),
                (self.a_primary, "/user/hand/left/input/trackpad/click"),
                (self.a_primary, "/user/hand/right/input/trackpad/click"),
                (self.a_secondary, "/user/hand/left/input/menu/click"),
                (self.a_secondary, "/user/hand/right/input/menu/click"),
                (self.a_haptic, "/user/hand/left/output/haptic"),
                (self.a_haptic, "/user/hand/right/output/haptic"),
            ],
            "/interaction_profiles/khr/simple_controller": [
                (self.a_pose, "/user/hand/left/input/grip/pose"),
                (self.a_pose, "/user/hand/right/input/grip/pose"),
                (self.a_trigger, "/user/hand/left/input/select/click"),
                (self.a_trigger, "/user/hand/right/input/select/click"),
                (self.a_secondary, "/user/hand/left/input/menu/click"),
                (self.a_secondary, "/user/hand/right/input/menu/click"),
                (self.a_haptic, "/user/hand/left/output/haptic"),
                (self.a_haptic, "/user/hand/right/output/haptic"),
            ],
        }
        for profile, binds in profiles.items():
            try:
                suggested = (xr.ActionSuggestedBinding * len(binds))(
                    *[xr.ActionSuggestedBinding(action=a, binding=p(path)) for a, path in binds]
                )
                xr.suggest_interaction_profile_bindings(
                    instance=inst,
                    suggested_bindings=xr.InteractionProfileSuggestedBinding(
                        interaction_profile=p(profile),
                        count_suggested_bindings=len(binds),
                        suggested_bindings=suggested,
                    ),
                )
            except xr.XrException:
                pass  # runtime may not know this profile; others still apply

        self.hand_spaces = {}
        for name, path in (
            ("left", self.hand_paths[0]),
            ("right", self.hand_paths[1]),
        ):
            self.hand_spaces[name] = xr.create_action_space(
                session=ctx.session,
                create_info=xr.ActionSpaceCreateInfo(
                    action=self.a_pose,
                    subaction_path=path,
                ),
            )

    def _setup_gl(self) -> None:
        self._provider.make_current()
        vs = GL.glCreateShader(GL.GL_VERTEX_SHADER)
        GL.glShaderSource(vs, _VERT)
        GL.glCompileShader(vs)
        fs = GL.glCreateShader(GL.GL_FRAGMENT_SHADER)
        GL.glShaderSource(fs, _FRAG)
        GL.glCompileShader(fs)
        prog = GL.glCreateProgram()
        GL.glAttachShader(prog, vs)
        GL.glAttachShader(prog, fs)
        GL.glLinkProgram(prog)
        if not GL.glGetProgramiv(prog, GL.GL_LINK_STATUS):
            raise RuntimeError(GL.glGetProgramInfoLog(prog))
        self._prog = prog
        self._mvp_loc = GL.glGetUniformLocation(prog, "mvp")

        h = SCREEN_WIDTH * self.image_h / self.image_w
        x0, x1 = -SCREEN_WIDTH / 2, SCREEN_WIDTH / 2
        y0, y1 = -h / 2, h / 2
        verts = np.array(
            [
                # x, y, u, v
                [x0, y0, 0, 0],
                [x1, y0, 1, 0],
                [x1, y1, 1, 1],
                [x0, y0, 0, 0],
                [x1, y1, 1, 1],
                [x0, y1, 0, 1],
            ],
            dtype=np.float32,
        )
        self._vao = GL.glGenVertexArrays(1)
        vbo = GL.glGenBuffers(1)
        GL.glBindVertexArray(self._vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, verts.nbytes, verts, GL.GL_STATIC_DRAW)
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(0, 2, GL.GL_FLOAT, False, 16, ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(1)
        GL.glVertexAttribPointer(1, 2, GL.GL_FLOAT, False, 16, ctypes.c_void_p(8))
        GL.glBindVertexArray(0)

        # one texture per eye: the sim is rendered from two offset cameras so
        # the virtual screen carries real depth (key for grasp precision)
        self._tex = [GL.glGenTextures(1) for _ in range(2)]
        for tex in self._tex:
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glTexImage2D(
                GL.GL_TEXTURE_2D,
                0,
                GL.GL_RGB8,
                self.image_w,
                self.image_h,
                0,
                GL.GL_RGB,
                GL.GL_UNSIGNED_BYTE,
                None,
            )
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

        # screen model matrix: translate to SCREEN_CENTER, facing +z (viewer)
        self._model = np.eye(4)
        self._model[:3, 3] = SCREEN_CENTER

    # ---------------------------------------------------------------- input
    def _poll(self, display_time) -> dict:
        ctx = self.context
        active = xr.ActiveActionSet(action_set=ctx.default_action_set, subaction_path=xr.NULL_PATH)
        try:
            xr.sync_actions(
                session=ctx.session,
                sync_info=xr.ActionsSyncInfo(
                    count_active_action_sets=1,
                    active_action_sets=pointer(active),
                ),
            )
        except xr.exception.SessionNotFocused:
            # headset not on / dashboard open: input is paused, not fatal —
            # keep rendering and try again next frame
            with self._lock:
                for hand in self.hands.values():
                    hand.valid = False
            return self.hands
        with self._lock:
            for name, path in (
                ("left", self.hand_paths[0]),
                ("right", self.hand_paths[1]),
            ):
                hand = self.hands[name]
                hand.valid = False
                loc = xr.locate_space(
                    space=self.hand_spaces[name],
                    base_space=ctx.space,
                    time=display_time,
                )
                flags = loc.location_flags
                if not (
                    flags & xr.SPACE_LOCATION_POSITION_VALID_BIT and flags & xr.SPACE_LOCATION_ORIENTATION_VALID_BIT
                ):
                    continue
                hand.pos = np.array(
                    [
                        loc.pose.position.x,
                        loc.pose.position.y,
                        loc.pose.position.z,
                    ]
                )
                hand.rot = _quat_xyzw_to_mat(loc.pose.orientation)

                def fstate(action):
                    return xr.get_action_state_float(
                        ctx.session,
                        xr.ActionStateGetInfo(action=action, subaction_path=path),
                    )

                hand.trigger = float(fstate(self.a_trigger).current_state)
                hand.grip = float(fstate(self.a_squeeze).current_state)
                stick = xr.get_action_state_vector2f(
                    ctx.session,
                    xr.ActionStateGetInfo(action=self.a_stick, subaction_path=path),
                ).current_state
                hand.stick = np.array([stick.x, stick.y])
                hand.stick_click = bool(
                    xr.get_action_state_boolean(
                        ctx.session,
                        xr.ActionStateGetInfo(action=self.a_stick_click, subaction_path=path),
                    ).current_state
                )
                hand.a = bool(
                    xr.get_action_state_boolean(
                        ctx.session,
                        xr.ActionStateGetInfo(action=self.a_primary, subaction_path=path),
                    ).current_state
                )
                hand.b = bool(
                    xr.get_action_state_boolean(
                        ctx.session,
                        xr.ActionStateGetInfo(action=self.a_secondary, subaction_path=path),
                    ).current_state
                )
                hand.valid = True
        return self.hands

    # --------------------------------------------------------------- render
    def _upload(self, frames: tuple) -> None:
        self._provider.make_current()
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        for tex, rgb in zip(self._tex, frames):
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glTexSubImage2D(
                GL.GL_TEXTURE_2D,
                0,
                0,
                0,
                self.image_w,
                self.image_h,
                GL.GL_RGB,
                GL.GL_UNSIGNED_BYTE,
                np.ascontiguousarray(rgb),
            )
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

    def _draw_view(self, view, eye_index: int) -> None:
        GL.glClearColor(0.05, 0.05, 0.08, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glDisable(GL.GL_DEPTH_TEST)
        proj = _projection_from_fov(view.fov, 0.05, 100.0)
        view_mat = np.linalg.inv(_pose_to_mat4(view.pose))
        mvp = (proj @ view_mat @ self._model).astype(np.float32)
        GL.glUseProgram(self._prog)
        GL.glUniformMatrix4fv(self._mvp_loc, 1, GL.GL_TRUE, mvp)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._tex[min(eye_index, len(self._tex) - 1)])
        GL.glBindVertexArray(self._vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 6)
        GL.glBindVertexArray(0)
        GL.glUseProgram(0)

    # ------------------------------------------------------------ threading
    def start(self) -> None:
        """Run the XR frame loop at headset rate in a background thread.

        Physics/control stays in the caller's free-running loop (same feel as
        the gamepad demo); the compositor re-projects the virtual screen with
        a fresh head pose every headset frame, so head motion stays smooth
        even though the screen content updates at simulation rate.
        """
        self._provider.done_current()  # hand the GL context to the XR thread
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def get_hands(self) -> dict:
        """Thread-safe snapshot of the latest controller states."""
        out = {}
        with self._lock:
            for name, hand in self.hands.items():
                snap = HandState()
                snap.pos = hand.pos.copy()
                snap.rot = hand.rot.copy()
                snap.trigger = hand.trigger
                snap.grip = hand.grip
                snap.stick = hand.stick.copy()
                snap.stick_click = hand.stick_click
                snap.a = hand.a
                snap.b = hand.b
                snap.valid = hand.valid
                out[name] = snap
        return out

    def set_haptic(self, hand: str, amplitude: float) -> None:
        """Queue controller vibration (0..1); applied by the XR thread."""
        with self._lock:
            self._haptics[hand] = max(self._haptics[hand], float(np.clip(amplitude, 0.0, 1.0)))

    def _apply_haptics(self) -> None:
        with self._lock:
            pending = dict(self._haptics)
            for k in self._haptics:
                self._haptics[k] = 0.0
        for name, path in (
            ("left", self.hand_paths[0]),
            ("right", self.hand_paths[1]),
        ):
            amp = pending.get(name, 0.0)
            if amp < 0.05:
                continue
            vib = xr.HapticVibration(
                duration=150_000_000,  # one-shot 150 ms pulse per new contact
                frequency=getattr(xr, "FREQUENCY_UNSPECIFIED", 0),
                amplitude=amp,
            )
            try:
                xr.apply_haptic_feedback(
                    session=self.context.session,
                    haptic_action_info=xr.HapticActionInfo(action=self.a_haptic, subaction_path=path),
                    haptic_feedback=cast(byref(vib), POINTER(xr.HapticBaseHeader)),
                )
                if not getattr(self, "_haptic_ok_logged", False):
                    self._haptic_ok_logged = True
                    print("[vr_openxr] haptics active (first pulse sent)")
            except xr.XrException as exc:
                if not getattr(self, "_haptic_err_logged", False):
                    self._haptic_err_logged = True
                    print(f"[vr_openxr] haptic feedback unavailable: {exc!r}")

    def submit(self, left_rgb: np.ndarray, right_rgb: np.ndarray | None = None) -> None:
        """Queue the latest sim image(s) for the virtual screen (keeps newest).

        With two images (stereo pair from horizontally offset cameras) each
        eye gets its own view and the screen shows real depth; with one, both
        eyes share it (mono).
        """
        with self._lock:
            self._pending_frame = (
                left_rgb,
                right_rgb if right_rgb is not None else left_rgb,
            )

    def _loop(self) -> None:
        try:
            for frame_state in self.context.frame_loop():
                if self._stop:
                    break
                with self._lock:
                    frame = self._pending_frame
                    self._pending_frame = None
                if frame is not None:
                    self._upload(frame)
                hands = self._poll(frame_state.predicted_display_time)
                del hands  # updated in place under lock inside _poll
                self._apply_haptics()
                for eye_index, view in enumerate(self.context.view_loop(frame_state)):
                    self._draw_view(view, eye_index)
        except BaseException as exc:  # surfaced to the control loop
            self.error = exc
