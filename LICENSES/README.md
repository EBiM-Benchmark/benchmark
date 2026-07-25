# The Isaac Lab copyright years differ on purpose — nothing here needs fixing

Four places in this repository state an Isaac Lab copyright year, and they do not all
agree. That is correct and deliberate. Do not "fix" it.

| Location | Year | What it is |
| --- | --- | --- |
| `LICENSES/BSD-3-Clause.txt` line 1 | `2022-2025` | Upstream's `LICENSE`, copied verbatim |
| `pyproject.toml` line 1 | `2022-2026` | Upstream's own file header, inherited |
| `.vscode/tools/setup_vscode.py` line 1 | `2022-2026` | Upstream's own file header, inherited |
| `NOTICE`, the paragraph beginning "This repository also incorporates portions of Isaac Lab" (line 12) | `2022-2026` | Describes the two headers above |

## Why

Upstream is internally inconsistent at a single commit. At `isaac-sim/IsaacLab`
`0916ea3c0f126821ef1783c7119d248834fc8d0b` — the commit Task 1 pins — its `LICENSE` reads
`2022-2025` while its own `pyproject.toml` and `.vscode/tools/setup_vscode.py` headers read
`2022-2026`. We inherit that inconsistency because we copied both sides exactly:

- `LICENSES/BSD-3-Clause.txt` is git blob `dee9ba551f428dd44471e7ee461528374233ad3c` —
  byte-identical to `isaac-sim/IsaacLab` `LICENSE` at that pinned commit, at tag `v2.3.2`,
  and at `main`. Reproducing a third party's license text unaltered is exactly what the
  BSD-3-Clause redistribution condition requires.
- The two headers are character-identical to upstream's and were inherited rather than
  written here. `git log -L 1,1:pyproject.toml` and
  `git log -L 1,1:.vscode/tools/setup_vscode.py` show the last change to each line
  predates this benchmark.
- `NOTICE` says `2022-2026` because that paragraph describes the *incorporated portions* —
  those two headers — and not the license text.

## Do not reconcile these

Editing `LICENSES/BSD-3-Clause.txt` to `2022-2026` would fabricate upstream's license text.
Editing `NOTICE` to `2022-2025` would describe the incorporated material less accurately.
Editing the headers would make them diverge from upstream.

## When this stops holding

This holds while Task 1 pins `0916ea3c`. Upstream last modified its `LICENSE` on
2025-06-05 (UTC) and has not touched it since, so the pin can move without the text
changing — but it may not. **On any change to the source pin, re-derive the blob against
the new ref before applying anything.** If upstream's `LICENSE` has changed, copying the
new text verbatim and re-syncing `NOTICE` becomes required, not forbidden.

---

This file is the canonical statement. Issue
[#27](https://github.com/EBiM-Benchmark/benchmark/issues/27) and its
[correction comment](https://github.com/EBiM-Benchmark/benchmark/issues/27#issuecomment-5078607110)
are history; if they and this file ever disagree, this file governs.
