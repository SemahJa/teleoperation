#!/usr/bin/env python3
"""Format maintained ROS, Python tooling and MATLAB code; --check never edits."""

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGES = (
    "quest_teleop",
    "quest_teleop_interfaces",
    "recording",
    "safety_filter",
    "teleop_bringup",
    "teleop_controllers",
    "teleop_gripper",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Report formatting drift")
    args = parser.parse_args()
    roots = [ROOT / "src" / name for name in PACKAGES] + [ROOT / "scripts"]
    files = sorted(path for root in roots for path in root.rglob("*") if path.is_file())
    cpp = [str(p) for p in files if p.suffix in {".cpp", ".hpp", ".h"}]
    python = [str(p) for p in files if p.suffix == ".py"]
    commands = [
        (
            "C++",
            [
                [
                    "clang-format",
                    *(["--dry-run", "--Werror"] if args.check else ["-i"]),
                    *cpp,
                ]
            ],
        ),
        # One file per invocation avoids process pools in restricted containers.
        (
            "Python",
            [
                [
                    sys.executable,
                    "-m",
                    "black",
                    "--quiet",
                    *(["--check"] if args.check else []),
                    path,
                ]
                for path in python
            ],
        ),
        (
            "MATLAB",
            [
                [
                    sys.executable,
                    "-m",
                    "miss_hit_core.mh_style",
                    *([] if args.check else ["--fix"]),
                    "analysis/matlab",
                ]
            ],
        ),
    ]
    failed = False
    for language, group in commands:
        print(
            f"Checking {language}" if args.check else f"Formatting {language}",
            flush=True,
        )
        for command in group:
            try:
                failed |= subprocess.run(command, cwd=ROOT, check=False).returncode != 0
            except FileNotFoundError as error:
                parser.exit(2, f"{error}. See docs/DEVELOPMENT.md for tool setup.\n")
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
