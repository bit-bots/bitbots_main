#!/usr/bin/env python3

"""Measure clean and incremental workspace build times."""

import argparse
import json
import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from statistics import fmean

DEFAULT_SOURCE = Path("src/bitbots_motion/bitbots_head_mover/src/move_head.cpp")
PACKAGES_TO_SKIP = ("zed_wrapper", "zed_components", "zed_msgs", "zed_ros2")
COMMON_CMAKE_ARGS = (
    "-DCMAKE_C_FLAGS=-Wall -Wextra -Werror -Wno-error=pedantic",
    "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Werror",
)


@dataclass(frozen=True)
class Experiment:
    """One build-system configuration in the repeated benchmark suite."""

    name: str
    label: str
    cmake_args: tuple[str, ...]
    uses_ccache: bool = False


EXPERIMENTS = (
    Experiment("baseline", "Unix Makefiles", ("-G", "Unix Makefiles")),
    Experiment("ninja", "Ninja", ("-G", "Ninja")),
    Experiment(
        "ccache",
        "Ninja + ccache",
        (
            "-G",
            "Ninja",
            "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
            "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
        ),
        uses_ccache=True,
    ),
    Experiment(
        "robot-profile",
        "Robot profile",
        (
            "-G",
            "Ninja",
            "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
            "-DBUILD_TESTING=OFF",
            "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
            "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
        ),
        uses_ccache=True,
    ),
)


def run_timed(command: list[str], env: dict[str, str] | None = None) -> dict[str, float | int | list[str]]:
    """Run a command and return its duration and exit status."""
    start = time.perf_counter()
    result = subprocess.run(command, check=False, env=env)
    return {
        "command": command,
        "duration_seconds": time.perf_counter() - start,
        "return_code": result.returncode,
    }


def append_single_summary(results: dict[str, object], summary_path: Path) -> None:
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


def append_suite_summary(results: dict[str, object], summary_path: Path) -> None:
    """Append all repeated experiment results to a GitHub Actions step summary."""
    experiments = results["experiments"]
    assert isinstance(experiments, list)

    with summary_path.open("a", encoding="utf-8") as summary:
        summary.write("## Repeated build benchmark\n\n")
        summary.write("| Experiment | Repetition | Clean | Incremental |\n")
        summary.write("| --- | ---: | ---: | ---: |\n")
        for experiment in experiments:
            assert isinstance(experiment, dict)
            runs = experiment["runs"]
            assert isinstance(runs, list)
            for run in runs:
                assert isinstance(run, dict)
                clean = run["clean_build"]
                incremental = run["incremental_build"]
                assert isinstance(clean, dict)
                assert isinstance(incremental, dict)
                summary.write(
                    f"| {experiment['label']} | {run['repetition']} "
                    f"| {clean['duration_seconds']:.2f} s "
                    f"| {incremental['duration_seconds']:.2f} s |\n"
                )
            averages = experiment["averages"]
            assert isinstance(averages, dict)
            summary.write(
                f"| **{experiment['label']}** | **mean** "
                f"| **{averages['clean_seconds']:.2f} s** "
                f"| **{averages['incremental_seconds']:.2f} s** |\n"
            )
        summary.write(f"\nTouched source: `{results['touched_source']}`\n")


def experiment_build_command(environment: str, experiment: Experiment) -> list[str]:
    """Construct the exact colcon command for an experiment."""
    return [
        "pixi",
        "run",
        "-e",
        environment,
        "colcon",
        "build",
        "--symlink-install",
        "--packages-skip",
        *PACKAGES_TO_SKIP,
        "--cmake-args",
        *experiment.cmake_args,
        *COMMON_CMAKE_ARGS,
    ]


def run_experiment(
    experiment: Experiment,
    repetitions: int,
    environment: str,
    source: Path,
    cache_root: Path,
) -> dict[str, object]:
    """Run one configuration repeatedly while retaining its isolated compiler cache."""
    command = experiment_build_command(environment, experiment)
    experiment_environment = os.environ.copy()
    if experiment.uses_ccache:
        experiment_environment["CCACHE_DIR"] = str(cache_root / experiment.name)

    runs = []
    for repetition in range(1, repetitions + 1):
        subprocess.run(
            ["pixi", "run", "-e", environment, "clean"],
            check=True,
            env=experiment_environment,
        )
        clean_build = run_timed(command, experiment_environment)
        os.utime(source)
        incremental_build = run_timed(command, experiment_environment)
        runs.append(
            {
                "repetition": repetition,
                "clean_build": clean_build,
                "incremental_build": incremental_build,
            }
        )

    return {
        "name": experiment.name,
        "label": experiment.label,
        "uses_ccache": experiment.uses_ccache,
        "command": command,
        "runs": runs,
        "averages": {
            "clean_seconds": fmean(run["clean_build"]["duration_seconds"] for run in runs),
            "incremental_seconds": fmean(run["incremental_build"]["duration_seconds"] for run in runs),
        },
    }


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
        "--suite",
        action="store_true",
        help="Run all build-system experiments instead of one Pixi task.",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=1,
        help="Number of clean and incremental measurements per suite experiment.",
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
    if args.repetitions < 1:
        raise ValueError("Repetitions must be at least one")

    if args.suite:
        runner_temp = os.environ.get("RUNNER_TEMP")
        cache_root = (
            Path(runner_temp) / "bitbots-build-benchmark-ccache"
            if runner_temp
            else Path(".ccache/build-benchmark-suite")
        )
        shutil.rmtree(cache_root, ignore_errors=True)
        suite_results = [
            run_experiment(
                experiment,
                args.repetitions,
                args.environment,
                args.source,
                cache_root,
            )
            for experiment in EXPERIMENTS
        ]
        results = {
            "created_at": datetime.now(UTC).isoformat(),
            "environment": args.environment,
            "repetitions": args.repetitions,
            "touched_source": str(args.source),
            "experiments": suite_results,
        }
        args.output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

        summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
        if summary_path:
            append_suite_summary(results, Path(summary_path))

        return max(
            build["return_code"]
            for experiment in suite_results
            for run in experiment["runs"]
            for build in (run["clean_build"], run["incremental_build"])
        )

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
        append_single_summary(results, Path(summary_path))

    return max(clean_build["return_code"], incremental_build["return_code"])


if __name__ == "__main__":
    raise SystemExit(main())
