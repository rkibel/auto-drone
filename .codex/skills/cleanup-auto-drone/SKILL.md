---
name: cleanup-auto-drone
description: Clean and self-correct this repository by finding dead code, stale references, obsolete 2D simulation paths, generated artifacts, unused package wiring, and documentation drift. Use when Codex is asked to tidy the auto-drone codebase or remove dead code.
---

# Cleanup Auto Drone

## Overview

Use this skill to keep the repository lean as the project moves from the early 2D prototype to the current 3D voxel active-mapping stack. It pairs a deterministic hygiene scanner with a self-correction loop.

## Quick Start

From the repository root, run:

```bash
python3 .codex/skills/cleanup-auto-drone/scripts/dead_code_audit.py --repo .
```

Then fix every actionable finding and rerun the audit. After cleanup passes, run the validation skill:

```bash
python3 .codex/skills/validate-auto-drone/scripts/validate_auto_drone.py --repo .
```

## What To Remove

Remove code and references that no longer serve the 3D simulator path:

- old 2D simulator modules, tests, launch files, configs, and entry points
- imports of removed modules such as `auto_drone.core`, `auto_drone.render`, or `auto_drone.sim_node`
- generated runtime output committed into source directories, such as frame dumps, videos, logs, build products, caches, and bytecode
- package metadata that names deleted launch/config files or deleted console scripts
- README or docs instructions that point to obsolete commands

## What To Preserve

Do not delete source that forms the simulator-transition boundary unless it is truly unused and tests confirm removal:

- `geometry3d.py`
- `sensing3d.py`
- `mapping3d.py`
- `odometry3d.py`
- `slam3d.py`
- `core3d.py`
- `render3d.py`
- `headless_3d_runner.py`
- `png.py`
- `video.py`
- `test/test_core3d.py`

Keep generated outputs ignored rather than tracked. It is fine for `build/`, `install/`, `log/`, and `frames3d/` to exist locally after runs, but they should not be treated as source.

## Self-Correction Loop

1. Run the dead-code audit.
2. Inspect each finding and verify it is not an intentional live path.
3. Remove stale code with the smallest coherent change.
4. Update docs, setup metadata, and tests in the same pass.
5. Run the dead-code audit again.
6. Run the validation skill.
7. Repeat until both checks pass.

If the audit flags a false positive, prefer tightening the audit script or adding an explicit allowlist entry over ignoring the result in prose.

## Cleanup Judgment

Be conservative with behavior and aggressive with dead surface area. If a module is imported only by tests, decide whether it represents desired behavior or obsolete compatibility. If it represents desired behavior, keep it and make the test meaningful. If it exists only to avoid breaking old commands, remove it and update the docs.
