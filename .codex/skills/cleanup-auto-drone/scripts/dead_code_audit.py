#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path


TEXT_SUFFIXES = {
    ".cfg",
    ".ini",
    ".md",
    ".py",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}

IGNORED_DIRS = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "frames",
    "frames3d",
}

OBSOLETE_TEXT_ALLOWLIST = {
    ".codex/skills/cleanup-auto-drone/SKILL.md",
    ".codex/skills/cleanup-auto-drone/scripts/dead_code_audit.py",
}

OBSOLETE_PATTERNS = {
    "2d module import": re.compile(r"\bauto_drone\.(core|render|sim_node|headless_runner)\b"),
    "2d class or runner name": re.compile(r"\b(Pose2D|BeliefMap|ActiveMappingSim\b|headless_sim_node|headless_sim)\b"),
    "obsolete 2d test": re.compile(r"\btest_core2d\b"),
}

EXPECTED_ENTRY_POINTS = {"gazebo_lidar_bridge", "headless_3d_runner"}
GENERATED_SUFFIXES = {".pyc", ".pyo", ".mp4", ".log"}
GENERATED_NAMES = {"__pycache__", ".pytest_cache"}


@dataclass
class Finding:
    kind: str
    path: str
    detail: str


def iter_source_files(repo: Path) -> list[Path]:
    files: list[Path] = []
    for root, dirs, names in os.walk(repo):
        root_path = Path(root)
        rel_parts = root_path.relative_to(repo).parts if root_path != repo else ()
        if any(part in IGNORED_DIRS for part in rel_parts):
            dirs[:] = []
            continue
        dirs[:] = [name for name in dirs if name not in IGNORED_DIRS and not name.startswith(".mypy_cache")]
        for name in names:
            path = root_path / name
            if path.suffix in TEXT_SUFFIXES:
                files.append(path)
    return files


def tracked_files(repo: Path) -> set[Path]:
    proc = subprocess.run(
        ["git", "ls-files"],
        cwd=repo,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    if proc.returncode != 0:
        return set()
    return {repo / line for line in proc.stdout.splitlines() if line}


def audit_obsolete_text(repo: Path) -> list[Finding]:
    findings: list[Finding] = []
    for path in iter_source_files(repo):
        rel = path.relative_to(repo)
        if str(rel) in OBSOLETE_TEXT_ALLOWLIST:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for idx, line in enumerate(text.splitlines(), start=1):
            for label, pattern in OBSOLETE_PATTERNS.items():
                if pattern.search(line):
                    findings.append(Finding(label, str(rel), f"line {idx}: {line.strip()}"))
    return findings


def audit_removed_files(repo: Path) -> list[Finding]:
    findings: list[Finding] = []
    forbidden_paths = [
        "auto_drone/core.py",
        "auto_drone/render.py",
        "auto_drone/headless_runner.py",
        "auto_drone/sim_node.py",
        "test/test_core2d.py",
        "launch/headless_sim.launch.py",
        "config/sim.yaml",
    ]
    for rel in forbidden_paths:
        if (repo / rel).exists():
            findings.append(Finding("obsolete file still exists", rel, "remove or justify the old 2D path"))
    return findings


def audit_generated_tracked(repo: Path) -> list[Finding]:
    findings: list[Finding] = []
    for path in tracked_files(repo):
        rel = path.relative_to(repo)
        if any(part in GENERATED_NAMES for part in rel.parts) or path.suffix in GENERATED_SUFFIXES:
            findings.append(Finding("generated file tracked", str(rel), "remove from source control and keep ignored"))
    return findings


def audit_entry_points(repo: Path) -> list[Finding]:
    setup_py = repo / "setup.py"
    if not setup_py.exists():
        return [Finding("missing setup.py", "setup.py", "ROS Python package metadata is missing")]

    text = setup_py.read_text(encoding="utf-8", errors="replace")
    names = set(re.findall(r"^\s*['\"]([A-Za-z0-9_.-]+)\s*=", text, flags=re.MULTILINE))
    findings: list[Finding] = []
    stale = names - EXPECTED_ENTRY_POINTS
    missing = EXPECTED_ENTRY_POINTS - names
    for name in sorted(stale):
        findings.append(Finding("unexpected console script", "setup.py", name))
    for name in sorted(missing):
        findings.append(Finding("missing console script", "setup.py", name))
    return findings


def audit_empty_legacy_dirs(repo: Path) -> list[Finding]:
    findings: list[Finding] = []
    for rel in ("launch", "config"):
        path = repo / rel
        if path.exists() and path.is_dir() and not any(path.iterdir()):
            findings.append(Finding("empty legacy directory", rel, "remove the empty directory if it is not needed"))
    return findings


def validate_repo_root(repo: Path) -> None:
    required = ["package.xml", "setup.py", "auto_drone"]
    missing = [name for name in required if not (repo / name).exists()]
    if missing:
        raise SystemExit(f"{repo} does not look like the auto-drone repo; missing: {', '.join(missing)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit auto-drone dead code and stale source surface.")
    parser.add_argument("--repo", default=".", help="Path to the auto-drone repository root.")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    validate_repo_root(repo)

    findings: list[Finding] = []
    findings.extend(audit_removed_files(repo))
    findings.extend(audit_obsolete_text(repo))
    findings.extend(audit_generated_tracked(repo))
    findings.extend(audit_entry_points(repo))
    findings.extend(audit_empty_legacy_dirs(repo))

    ok = not findings
    print(json.dumps({"ok": ok, "findings": [asdict(finding) for finding in findings]}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
