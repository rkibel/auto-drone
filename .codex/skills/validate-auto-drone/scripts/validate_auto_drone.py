#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path


LINE_RE = re.compile(
    r"step=(?P<step>\d+)\s+known=(?P<known>\d+\.\d+)\s+"
    r"uncertainty=(?P<uncertainty>\d+\.\d+)"
)


@dataclass
class CheckResult:
    name: str
    ok: bool
    details: str
    elapsed_s: float


def run_command(name: str, cmd: list[str], cwd: Path, timeout_s: int = 120) -> CheckResult:
    start = time.perf_counter()
    try:
        proc = subprocess.run(
            cmd,
            cwd=cwd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        elapsed = time.perf_counter() - start
        output = exc.stdout or ""
        return CheckResult(name, False, f"timed out after {timeout_s}s\n{output[-3000:]}", elapsed)

    elapsed = time.perf_counter() - start
    output = proc.stdout or ""
    if proc.returncode != 0:
        return CheckResult(name, False, output[-5000:], elapsed)
    return CheckResult(name, True, output[-3000:], elapsed)


def pytest_check(repo: Path, skip_colcon: bool) -> CheckResult:
    result = run_command("pytest", [sys.executable, "-m", "pytest", "-q"], repo)
    if result.ok:
        return result

    missing_pytest = "No module named pytest" in result.details
    if missing_pytest and shutil.which("colcon") and not skip_colcon:
        return CheckResult(
            "pytest",
            True,
            "skipped direct pytest: active Python has no pytest; colcon test will run the suite",
            result.elapsed_s,
        )
    return result


def parse_mapping_lines(output: str) -> list[tuple[int, float, float]]:
    rows: list[tuple[int, float, float]] = []
    for line in output.splitlines():
        match = LINE_RE.search(line)
        if match:
            rows.append(
                (
                    int(match.group("step")),
                    float(match.group("known")),
                    float(match.group("uncertainty")),
                )
            )
    return rows


def no_render_smoke(
    repo: Path,
    max_step_time: float,
    min_known_gain: float,
    min_uncertainty_drop: float,
) -> CheckResult:
    with tempfile.TemporaryDirectory(prefix="auto_drone_validate_") as tmp:
        tmp_path = Path(tmp)
        cmd = [
            sys.executable,
            "-m",
            "auto_drone.headless_3d_runner",
            "--max-steps",
            "80",
            "--render-every",
            "10000",
            "--output-dir",
            str(tmp_path / "frames"),
            "--video-path",
            str(tmp_path / "exploration3d.mp4"),
        ]
        start = time.perf_counter()
        proc = subprocess.run(
            cmd,
            cwd=repo,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=120,
        )
        elapsed = time.perf_counter() - start

    output = proc.stdout or ""
    if proc.returncode != 0:
        return CheckResult("3d no-render smoke", False, output[-5000:], elapsed)

    rows = parse_mapping_lines(output)
    if len(rows) < 2:
        return CheckResult(
            "3d no-render smoke",
            False,
            "runner completed but emitted too few metric lines\n" + output[-3000:],
            elapsed,
        )

    steps = max(1, rows[-1][0] - rows[0][0] + 1)
    avg_step_time = elapsed / steps
    known_gain = rows[-1][1] - rows[0][1]
    uncertainty_drop = rows[0][2] - rows[-1][2]

    failures: list[str] = []
    if avg_step_time > max_step_time:
        failures.append(f"avg_step_time={avg_step_time:.4f}s > {max_step_time:.4f}s")
    if known_gain < min_known_gain:
        failures.append(f"known_gain={known_gain:.4f} < {min_known_gain:.4f}")
    if uncertainty_drop < min_uncertainty_drop:
        failures.append(f"uncertainty_drop={uncertainty_drop:.4f} < {min_uncertainty_drop:.4f}")

    details = (
        f"steps={steps} avg_step_time={avg_step_time:.4f}s "
        f"known_gain={known_gain:.4f} uncertainty_drop={uncertainty_drop:.4f}\n"
        + output[-2000:]
    )
    if failures:
        details = "\n".join(failures) + "\n" + details
    return CheckResult("3d no-render smoke", not failures, details, elapsed)


def render_smoke(repo: Path) -> CheckResult:
    with tempfile.TemporaryDirectory(prefix="auto_drone_render_") as tmp:
        tmp_path = Path(tmp)
        frames = tmp_path / "frames"
        video = tmp_path / "exploration3d.mp4"
        cmd = [
            sys.executable,
            "-m",
            "auto_drone.headless_3d_runner",
            "--max-steps",
            "24",
            "--render-every",
            "6",
            "--output-dir",
            str(frames),
            "--video-path",
            str(video),
            "--fps",
            "8",
        ]
        start = time.perf_counter()
        proc = subprocess.run(
            cmd,
            cwd=repo,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=120,
        )
        elapsed = time.perf_counter() - start
        output = proc.stdout or ""
        final_png = frames / "final.png"
        failures: list[str] = []
        if proc.returncode != 0:
            failures.append("runner returned non-zero")
        if not final_png.exists() or final_png.stat().st_size == 0:
            failures.append("final.png was not created")
        if shutil.which("ffmpeg") and (not video.exists() or video.stat().st_size == 0):
            failures.append("ffmpeg is available but mp4 was not created")

    details = output[-3000:]
    if failures:
        details = "\n".join(failures) + "\n" + details
    return CheckResult("3d render smoke", not failures, details, elapsed)


def maybe_colcon(repo: Path, skip_colcon: bool) -> list[CheckResult]:
    if skip_colcon or not shutil.which("colcon"):
        return [CheckResult("colcon", True, "skipped: colcon unavailable or disabled", 0.0)]

    setup = Path("/opt/ros/humble/setup.bash")
    prefix = f"source {setup} && " if setup.exists() else ""
    return [
        run_command(
            "colcon build",
            ["bash", "-lc", prefix + "colcon build --event-handlers console_direct+"],
            repo,
            timeout_s=180,
        ),
        run_command(
            "colcon test",
            ["bash", "-lc", prefix + "colcon test --event-handlers console_direct+"],
            repo,
            timeout_s=180,
        ),
    ]


def validate_repo_root(repo: Path) -> None:
    required = ["package.xml", "setup.py", "auto_drone"]
    missing = [name for name in required if not (repo / name).exists()]
    if missing:
        raise SystemExit(f"{repo} does not look like the auto-drone repo; missing: {', '.join(missing)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate auto-drone correctness, speed, and mapping efficiency.")
    parser.add_argument("--repo", default=".", help="Path to the auto-drone repository root.")
    parser.add_argument("--max-step-time", type=float, default=0.08)
    parser.add_argument("--min-known-gain", type=float, default=0.03)
    parser.add_argument("--min-uncertainty-drop", type=float, default=0.03)
    parser.add_argument("--skip-colcon", action="store_true")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    validate_repo_root(repo)

    results: list[CheckResult] = [
        run_command("compileall", [sys.executable, "-m", "compileall", "auto_drone", "test"], repo),
        pytest_check(repo, args.skip_colcon),
        no_render_smoke(repo, args.max_step_time, args.min_known_gain, args.min_uncertainty_drop),
        render_smoke(repo),
    ]
    results.extend(maybe_colcon(repo, args.skip_colcon))

    ok = all(result.ok for result in results)
    print(json.dumps({"ok": ok, "results": [asdict(result) for result in results]}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
