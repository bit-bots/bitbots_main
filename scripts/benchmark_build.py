#!/usr/bin/env python3

"""Measure clean and incremental workspace build times."""

import argparse
import json
import os
import subprocess
import time
from datetime import UTC, datetime
from pathlib import Path

DEFAULT_SOURCE = Path("src/bitbots_motion/bitbots_head_mover/src/move_head.cpp")


def run_timed(command: list[str]) -> dict[str, float | int | list[str]]:
    """Run a command and return its duration and exit status."""
    start = time.perf_counter()
    result = subprocess.run(command, check=False)
    return {
        "command": command,
        "duration_seconds": time.perf_counter() - start,
        "return_code": result.returncode,
    }


def append_github_summary(results: dict[str, object], summary_path: Path) -> None:
    """Append the benchmark results to a GitHub Actions step summary."""
    clean = results["clean_build"]
    incremental = results["incremental_build"]
    assert isinstance(clean, dict)
    assert isinstance(incremental, dict)

    with summary_path.open("a", encoding="utf-8") as summary:
        summary.write("## Build benchmark\n\n")
        summary.write("| Build | Duration | Exit code |\n")
        summary.write("| --- | ---: | ---: |\n")
        summary.write(f"| Clean | {clean['duration_seconds']:.2f} s | {clean['return_code']} |\n")
        summary.write(f"| Incremental | {incremental['duration_seconds']:.2f} s | {incremental['return_code']} |\n\n")
        summary.write(f"Pixi task: `{results['task']}`\n\n")
        summary.write(f"Touched source: `{results['touched_source']}`\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--environment",
        default="default",
        help="Pixi environment containing the workspace build task.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build-benchmark.json"),
        help="JSON file that receives the benchmark results.",
    )
    parser.add_argument(
        "--task",
        default="build",
        help="Pixi task used for both measured builds.",
    )
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help="Existing C++ source to touch before the incremental build.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.source.is_file():
        raise FileNotFoundError(f"Incremental benchmark source does not exist: {args.source}")

    subprocess.run(
        ["pixi", "run", "-e", args.environment, "clean"],
        check=True,
    )
    build_command = ["pixi", "run", "-e", args.environment, args.task]
    clean_build = run_timed(build_command)

    # Updating the timestamp makes the build system reconsider one translation unit.
    # The file content stays unchanged, allowing compiler caches to reuse their output.
    os.utime(args.source)
    incremental_build = run_timed(build_command)

    results = {
        "created_at": datetime.now(UTC).isoformat(),
        "environment": args.environment,
        "task": args.task,
        "touched_source": str(args.source),
        "clean_build": clean_build,
        "incremental_build": incremental_build,
    }
    args.output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary_path:
        append_github_summary(results, Path(summary_path))

    return max(clean_build["return_code"], incremental_build["return_code"])


if __name__ == "__main__":
    raise SystemExit(main())
