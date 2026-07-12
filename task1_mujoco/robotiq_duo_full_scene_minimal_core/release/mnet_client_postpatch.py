"""Post-`colcon build` patches for the vendored mnet client, cross-platform.

Two things the vendored client (mnet_client-ros_2, which this repo does not
modify at the source level) needs; both re-apply after every `colcon build`
since that copies fresh files into the workspace's install directory:

1. `team_config.json`'s `file_dir` points at the container path `/ws/out`
   by default - point it at a real local directory instead. Written with
   plain UTF-8 (no BOM); on Windows, PowerShell's `Out-File`/`Set-Content`
   add one and break the client's JSON parsing, so this always goes
   through Python instead.
2. The client polls stdin with `select.select(...)`, which is POSIX-only
   and crashes on Windows (`WinError 10038`). Swaps it for `msvcrt.kbhit()`
   inline, at every call site, guarded by an `os.name == 'nt'` check -
   a no-op on Linux (still calls the original `select.select`), so this
   patch is safe to apply on every platform without branching the caller.

Called by setup_eval.bat / setup_eval.sh - not meant to be run by hand, but
safe to: every step here is idempotent (checks before it writes).

    python mnet_client_postpatch.py <workspace_dir> <output_dir>
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

# tolerant of the vendored client's own line-wrapping (some call sites split
# the trailing [0] onto its own line) - \s matches across newlines too
BARE_CALL = "select.select([sys.stdin], [], [], 0.0)[0]"
SELECT_PATTERN = re.compile(r"select\.select\(\[sys\.stdin\],\s*\[\],\s*\[\],\s*0\.0\)\[\s*0\s*\]")
WIN_REPLACEMENT = f"(__import__('msvcrt').kbhit() if __import__('os').name == 'nt' else {BARE_CALL})"
# matches a previously-applied WIN_REPLACEMENT (however the inner bare call
# was spaced/wrapped) so re-running this script is idempotent instead of
# nesting a new wrapper around an already-patched site
WRAPPED_PATTERN = re.compile(
    r"\(__import__\('msvcrt'\)\.kbhit\(\) if __import__\('os'\)\.name == 'nt' else "
    r"select\.select\(\[sys\.stdin\],\s*\[\],\s*\[\],\s*0\.0\)\[\s*0\s*\]\)"
)


def patch_team_config(workspace: Path, output_dir: Path) -> None:
    configs = list(workspace.glob("install/**/config/team_config.json"))
    if not configs:
        print(f"[setup] WARNING: team_config.json not found under {workspace}/install - build first")
        return
    for cfg in configs:
        data = json.loads(cfg.read_text(encoding="utf-8"))
        if data.get("file_dir") in (None, "/ws/out"):
            data["file_dir"] = str(output_dir)
            cfg.write_text(json.dumps(data, indent=4), encoding="utf-8")
            print(f"[setup] file_dir -> {output_dir}  ({cfg})")
        else:
            print(f"[setup] file_dir already customized ('{data['file_dir']}'), left alone  ({cfg})")


def patch_stdin(workspace: Path) -> None:
    targets = list(workspace.glob("install/**/clients/local_test_client.py"))
    targets += list(workspace.glob("install/**/clients/submission_client.py"))
    if not targets:
        print(f"[setup] WARNING: client source files not found under {workspace}/install - build first")
        return
    for f in targets:
        text = f.read_text(encoding="utf-8")
        # normalize back to bare calls first - loop in case an earlier buggy
        # run left nested wrappers - so the wrap pass below never double-wraps
        already = 0
        while True:
            text, k = WRAPPED_PATTERN.subn(BARE_CALL, text)
            already += k
            if k == 0:
                break
        text, n = SELECT_PATTERN.subn(WIN_REPLACEMENT, text)
        if n == 0 and already == 0:
            print(f"[setup] WARNING: expected select.select() pattern not found in {f.name} "
                  "(upstream client changed? report this)")
            continue
        f.write_text(text, encoding="utf-8")
        print(f"[setup] stdin fix applied ({n} call site(s), {already} already patched)  ({f.name})")


def main() -> None:
    if len(sys.argv) != 3:
        print("usage: mnet_client_postpatch.py <workspace_dir> <output_dir>")
        raise SystemExit(1)
    workspace = Path(sys.argv[1]).resolve()
    output_dir = Path(sys.argv[2]).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    patch_team_config(workspace, output_dir)
    patch_stdin(workspace)


if __name__ == "__main__":
    main()
