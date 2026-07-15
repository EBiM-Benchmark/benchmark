"""HTTP request handler and server for the browser controller frontend.

Serves static files from ../static/ and delegates all API calls to the
BrowserControllerBridge instance injected at construction time.
"""

import json
import mimetypes
import os
import threading
import time
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_STATIC_DIR = os.path.join(os.path.dirname(__file__), "..", "static")


class SliderRequestHandler(BaseHTTPRequestHandler):
    bridge = None

    def log_message(self, fmt, *args):
        return

    def _send_json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, rel_path, status=HTTPStatus.OK):
        abs_path = os.path.normpath(os.path.join(_STATIC_DIR, rel_path))
        # Prevent path traversal outside static dir
        if not abs_path.startswith(os.path.normpath(_STATIC_DIR)):
            self._send_json({"error": "forbidden"}, HTTPStatus.FORBIDDEN)
            return
        if not os.path.isfile(abs_path):
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        content_type, _ = mimetypes.guess_type(abs_path)
        if content_type is None:
            content_type = "application/octet-stream"
        with open(abs_path, "rb") as f:
            body = f.read()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, body, *, content_type: str, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        route = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if route in ("/", "/index.html"):
            self._send_file("index.html")
            return

        if route == "/monitor":
            self._send_file("monitor.html")
            return

        if route == "/topology":
            self._send_file("topology.html")
            return

        if route.startswith("/static/"):
            rel = route[len("/static/"):]
            self._send_file(rel)
            return

        if route == "/api/topics":
            if self.bridge is None:
                self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send_json({"topics": self.bridge.get_topic_overview(), "timestamp": time.time()})
            return

        if route == "/api/joints":
            if self.bridge is None:
                self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send_json(self.bridge.get_snapshot())
            return

        if route == "/api/cameras/frame":
            if self.bridge is None:
                self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            camera_key = ""
            if "camera_key" in query and query["camera_key"]:
                camera_key = str(query["camera_key"][0] or "")
            preview = self.bridge.get_camera_preview(camera_key)
            if preview is None:
                self._send_json(
                    {"error": f"preview unavailable for {camera_key}"},
                    HTTPStatus.NOT_FOUND,
                )
                return
            self._send_bytes(
                bytes(preview["data"]),
                content_type=str(preview["content_type"]),
            )
            return

        if route == "/api/control_mode":
            if self.bridge is None:
                self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send_json(self.bridge.get_control_mode())
            return

        if route == "/api/topology":
            if self.bridge is None:
                self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send_json(self.bridge.get_topology_snapshot())
            return

        if route == "/api/monitor":
            if self.bridge is None:
                self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self._send_json({"topics": self.bridge.get_monitor_snapshot(), "timestamp": time.time()})
            return

        self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        if self.path not in (
            "/api/command",
            "/api/reset",
            "/api/cameras",
            "/api/control_mode",
        ):
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return

        if self.bridge is None:
            self._send_json({"error": "bridge not initialized"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
            payload = json.loads(raw.decode("utf-8"))

            if self.path == "/api/reset":
                command_topic = payload.get("command_topic")
                if command_topic is not None and not isinstance(command_topic, str):
                    self._send_json(
                        {"status": "error", "message": "command_topic must be a string"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                ok, message = self.bridge.reset_targets(command_topic=command_topic)
                if not ok:
                    self._send_json({"status": "error", "message": message}, HTTPStatus.BAD_REQUEST)
                    return
                self._send_json({"status": "ok", "message": message}, HTTPStatus.OK)
                return

            if self.path == "/api/control_mode":
                mode = payload.get("mode")
                if not isinstance(mode, str):
                    self._send_json(
                        {"status": "error", "message": "mode must be a string"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                ok, message = self.bridge.set_control_mode(mode)
                if not ok:
                    self._send_json({"status": "error", "message": message}, HTTPStatus.BAD_REQUEST)
                    return
                self._send_json({"status": "ok", "message": message}, HTTPStatus.OK)
                return

            if self.path == "/api/cameras":
                camera_key = payload.get("camera_key")
                enabled = payload.get("enabled")
                if not isinstance(camera_key, str):
                    self._send_json(
                        {"status": "error", "message": "camera_key must be a string"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                if not isinstance(enabled, bool):
                    self._send_json(
                        {"status": "error", "message": "enabled must be a boolean"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                ok, message = self.bridge.set_camera_enabled(camera_key, enabled)
                if not ok:
                    self._send_json({"status": "error", "message": message}, HTTPStatus.BAD_REQUEST)
                    return
                self._send_json({"status": "ok", "message": message}, HTTPStatus.OK)
                return

            command_topic = payload.get("command_topic", "")
            positions = payload.get("positions")
            if positions is None and "joint_name" in payload and "position" in payload:
                positions = {payload["joint_name"]: payload["position"]}

            ok, message = self.bridge.set_targets(command_topic, positions)
            if not ok:
                self._send_json({"status": "error", "message": message}, HTTPStatus.BAD_REQUEST)
                return

            self._send_json({"status": "queued"}, HTTPStatus.ACCEPTED)
        except json.JSONDecodeError:
            self._send_json({"status": "error", "message": "invalid json"}, HTTPStatus.BAD_REQUEST)
        except Exception as error:
            self._send_json(
                {"status": "error", "message": str(error)},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )


class SliderServer:
    def __init__(self, host: str, port: int, bridge):
        handler = type("DynamicSliderRequestHandler", (SliderRequestHandler,), {})
        handler.bridge = bridge
        self._server = ThreadingHTTPServer((host, port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
