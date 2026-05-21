---
name: validate-auto-drone
description: Validate and self-correct this repository's 3D voxel mapping stack. Use when Codex is asked to run tests, check real-time behavior, assess mapping efficiency, verify smoke runs, debug regressions, or keep the auto-drone SLAM-transition codebase healthy after changes.
---

# Validate Auto Drone

## Overview

Use this skill to keep the auto-drone project correct, fast, and mapping-efficient as the stack evolves toward simulator and SLAM integration. The workflow combines unit tests, smoke runs, timing checks, coverage/uncertainty metrics, render checks, and a bounded self-correction loop.

## Quick Start

From the repository root, run:

```bash
python3 .codex/skills/validate-auto-drone/scripts/validate_auto_drone.py --repo .
```

Treat any failure as actionable. Fix the smallest relevant code path, then rerun the validator. Repeat until it passes or three correction attempts have failed for the same issue.

## Validation Scope

The validator checks:

- Python import/bytecode health with `python3 -m compileall auto_drone test`
- unit tests with `python3 -m pytest -q`, or with `colcon test` when the active shell Python does not have `pytest`
- a no-render 3D mapping smoke run to measure average step time, known-space gain, and uncertainty reduction
- a render/video smoke run to confirm the isometric visualization path still works
- `colcon build` and `colcon test` when ROS 2 tooling is available

Default performance expectations:

- average no-render step time below `0.08s`
- known-ratio gain at least `0.03` during the smoke run
- mean uncertainty drop at least `0.03` during the smoke run

These are smoke thresholds, not research benchmarks. If a threshold becomes too loose or too strict, prefer improving the test scenario or implementation before changing the limit.

## Self-Correction Loop

1. Run the bundled validator from the repo root.
2. Read the failing command, metric, and validator details.
3. Inspect the smallest relevant module before editing.
4. Patch the root cause, not just the symptom.
5. Rerun the validator.
6. Stop only when all checks pass, or when a real blocker needs user input.

Use this diagnosis guide:

- compile or import failures: check deleted-module references, package entry points, and stale public imports.
- unit test failures: preserve the intended active-mapping behavior unless the test is clearly obsolete.
- timing failures: inspect planner candidate counts, ray generation, repeated full-volume scans, render frequency, and cache use.
- mapping-efficiency failures: inspect range-frame integration, evidence weights, free-space carving, target scoring, and termination logic.
- render failures: inspect `render3d.py`, `png.py`, `video.py`, output paths, and optional `ffmpeg` availability.
- ROS build failures: inspect `setup.py`, `package.xml`, resource markers, and launch/config references.

## Codebase Hygiene Rules

Keep the project 3D-first. Do not reintroduce the removed 2D simulator unless the user explicitly asks for a compatibility layer.

Preserve clear simulator-transition boundaries:

- `geometry3d.py`: pose, orientation, and ray geometry
- `sensing3d.py`: synthetic sensor generation
- `mapping3d.py`: belief updates from sensor-like frames
- `odometry3d.py`: pose-noise simulation
- `slam3d.py`: lightweight pose graph placeholder
- `core3d.py`: orchestration, planning, and stepping
- `render3d.py`: visualization only

Generated artifacts such as frames, videos, `build/`, `install/`, and `log/` are not source. Do not treat them as code health evidence except when validating output creation.
