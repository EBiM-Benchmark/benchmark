"""On-board one-time-code display (ManipulationNet submission requirement).

The mnet server generates a one-time code when a submission starts and
requires it to be VISIBLE IN THE CAMERA VIEW throughout the run (the
physical-world instruction is "write it on paper and place it in view").
The client never checks the image automatically — the code is bound to the
video cryptographically (frame/video hashes) and its presence is verified by
the server-side review — but it still has to be in frame.

The sim equivalent is the ``code_plate`` body in the scene XML: a white
plate on the table's front strip inside the overhead camera frame, carrying
8 character slots of 5x7 dot-matrix geoms. Dots are plain model geoms whose
rgba alpha is toggled here, so the text shows up in every render context
(desktop viewer, HMD view, evidence camera) with no texture uploads.

Set the code with --display-code at startup, or at runtime by typing
``code <TEXT>`` into the sim terminal (see stdin_command_listener).
"""

from __future__ import annotations

import queue
import sys
import threading

import mujoco

from . import log

# 5x7 dot-matrix font, rows top to bottom, '#' = dot on
_FONT = {
    "0": ["#####", "#...#", "#..##", "#.#.#", "##..#", "#...#", "#####"],
    "1": ["..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###."],
    "2": ["#####", "....#", "....#", "#####", "#....", "#....", "#####"],
    "3": ["#####", "....#", "....#", ".####", "....#", "....#", "#####"],
    "4": ["#...#", "#...#", "#...#", "#####", "....#", "....#", "....#"],
    "5": ["#####", "#....", "#....", "#####", "....#", "....#", "#####"],
    "6": ["#####", "#....", "#....", "#####", "#...#", "#...#", "#####"],
    "7": ["#####", "....#", "...#.", "..#..", ".#...", ".#...", ".#..."],
    "8": ["#####", "#...#", "#...#", "#####", "#...#", "#...#", "#####"],
    "9": ["#####", "#...#", "#...#", "#####", "....#", "....#", "#####"],
    "A": [".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"],
    "B": ["####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."],
    "C": [".####", "#....", "#....", "#....", "#....", "#....", ".####"],
    "D": ["####.", "#...#", "#...#", "#...#", "#...#", "#...#", "####."],
    "E": ["#####", "#....", "#....", "####.", "#....", "#....", "#####"],
    "F": ["#####", "#....", "#....", "####.", "#....", "#....", "#...."],
    "G": [".####", "#....", "#....", "#.###", "#...#", "#...#", ".###."],
    "H": ["#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"],
    "I": [".###.", "..#..", "..#..", "..#..", "..#..", "..#..", ".###."],
    "J": ["..###", "...#.", "...#.", "...#.", "...#.", "#..#.", ".##.."],
    "K": ["#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"],
    "L": ["#....", "#....", "#....", "#....", "#....", "#....", "#####"],
    "M": ["#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"],
    "N": ["#...#", "##..#", "#.#.#", "#..##", "#...#", "#...#", "#...#"],
    "O": [".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."],
    "P": ["####.", "#...#", "#...#", "####.", "#....", "#....", "#...."],
    "Q": [".###.", "#...#", "#...#", "#...#", "#.#.#", "#..#.", ".##.#"],
    "R": ["####.", "#...#", "#...#", "####.", "#.#..", "#..#.", "#...#"],
    "S": [".####", "#....", "#....", ".###.", "....#", "....#", "####."],
    "T": ["#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."],
    "U": ["#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."],
    "V": ["#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.."],
    "W": ["#...#", "#...#", "#...#", "#.#.#", "#.#.#", "##.##", "#...#"],
    "X": ["#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"],
    "Y": ["#...#", "#...#", ".#.#.", "..#..", "..#..", "..#..", "..#.."],
    "Z": ["#####", "....#", "...#.", "..#..", ".#...", "#....", "#####"],
    "-": [".....", ".....", ".....", "#####", ".....", ".....", "....."],
    " ": [".....", ".....", ".....", ".....", ".....", ".....", "....."],
}

N_SLOTS = 8


class CodeDisplay:
    """Drives the code_plate dot-matrix. Missing plate degrades to a no-op."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self._dots: list[list[list[int]]] = []  # [slot][row][col] -> geom id
        ok = True
        for i in range(N_SLOTS):
            slot = []
            for r in range(7):
                row = []
                for k in range(5):
                    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"code_c{i}_r{r}_k{k}")
                    if gid < 0:
                        ok = False
                    row.append(gid)
                slot.append(row)
            self._dots.append(slot)
        self.available = ok
        if not ok:
            log("[code] code_plate geoms missing from the model; code display disabled")

    def show(self, text: str) -> None:
        """Render up to 8 characters (A-Z, 0-9, '-') on the plate."""
        if not self.available:
            return
        text = text.strip().upper()[:N_SLOTS]
        padded = text.center(N_SLOTS)
        for i, ch in enumerate(padded):
            pattern = _FONT.get(ch, _FONT[" "])
            for r in range(7):
                for k in range(5):
                    on = pattern[r][k] == "#"
                    self.model.geom_rgba[self._dots[i][r][k]][3] = 1.0 if on else 0.0
        log(f"[code] displaying on the board plate: '{text}'")


def stdin_command_listener() -> queue.Queue[str]:
    """Background stdin reader: typing ``code ABC123`` into the sim terminal
    queues the text for the main loop (viewer windows can't take text input).
    Returns the queue; the thread dies with the process."""
    q: queue.Queue[str] = queue.Queue()

    def reader() -> None:
        for line in sys.stdin:
            parts = line.strip().split(maxsplit=1)
            if len(parts) == 2 and parts[0].lower() == "code":
                q.put(parts[1])

    threading.Thread(target=reader, daemon=True, name="stdin-code").start()
    return q
