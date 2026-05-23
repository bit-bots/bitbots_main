"""Performance timing decorator for simulation benchmarking.

This module provides timing decorators and metrics collection for comparing
simulator performance. It collects:
- Function execution times
- CPU usage (user/system time)
- Real-time factor (RTF) - calculated from wall clock vs simulation time
- Simulation time

Data is written both as a human-readable summary and as a CSV for graphing.

Usage:
    from bitbots_utils.perf_timer import timed, set_sim_time, configure_output

    # Configure output paths (call once at startup)
    configure_output("/tmp/mujoco_perf")

    # Decorate functions to time
    @timed
    def my_function():
        ...

    # Update simulation time periodically (RTF calculated automatically)
    set_sim_time(current_sim_time)
"""

import atexit
import csv
import functools
import resource
import signal
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, TypeVar

F = TypeVar("F", bound=Callable)


@dataclass
class TimeSample:
    """A single time-series sample of performance metrics."""

    wall_time: float  # Wall clock time since start
    sim_time: float  # Simulation time
    rtf: float  # Real-time factor (sim_time_delta / wall_time_delta)
    cpu_user: float  # Cumulative user CPU time
    cpu_system: float  # Cumulative system CPU time
    total_calls: int  # Total function calls so far
    # Snapshot of per-function timing data at this sample point
    func_timing: dict[str, dict] = field(default_factory=dict)


@dataclass
class PerfData:
    """Central storage for all performance data."""

    # Configuration
    output_prefix: str = "/tmp/sim_perf"
    simulator_name: str = "simulator"

    # Timing data per function
    timing_data: dict = field(default_factory=lambda: defaultdict(lambda: {"total_time": 0.0, "call_count": 0}))

    # Time-series samples
    samples: list[TimeSample] = field(default_factory=list)

    # Current state
    start_time: float = field(default_factory=time.time)
    start_cpu: resource.struct_rusage = field(default_factory=lambda: resource.getrusage(resource.RUSAGE_SELF))
    last_write_count: int = 0

    # Simulation time tracking
    sim_time: float = 0.0
    last_sample_wall_time: float = 0.0
    last_sample_sim_time: float = 0.0

    # Sample interval (in total function calls)
    sample_interval: int = 1000


# Global instance
_perf = PerfData()


def configure_output(prefix: str, simulator_name: str = "simulator", sample_interval: int = 1000) -> None:
    """
    Configure output file paths and simulator name.

    Args:
        prefix: Path prefix for output files (e.g., "/tmp/mujoco_perf")
                Creates {prefix}.txt and {prefix}.csv
        simulator_name: Name to identify this simulator in reports
        sample_interval: How often to collect time-series samples (in function calls)
    """
    _perf.output_prefix = prefix
    _perf.simulator_name = simulator_name
    _perf.sample_interval = sample_interval
    # Reset timing state for fresh start
    _perf.start_time = time.time()
    _perf.start_cpu = resource.getrusage(resource.RUSAGE_SELF)
    _perf.last_sample_wall_time = 0.0
    _perf.last_sample_sim_time = 0.0


def set_sim_time(sim_time: float) -> None:
    """
    Update the current simulation time.

    RTF is calculated automatically from wall clock time vs simulation time.
    Call this periodically from your simulation step function.
    """
    _perf.sim_time = sim_time


# Backward compatibility alias
def set_rtf(rtf: float, sim_time: float = 0.0) -> None:
    """Deprecated: Use set_sim_time() instead. RTF is calculated automatically."""
    if sim_time > 0:
        _perf.sim_time = sim_time


def _collect_sample() -> None:
    """Collect a time-series sample of current metrics."""
    now = time.time()
    wall_elapsed = now - _perf.start_time
    cpu = resource.getrusage(resource.RUSAGE_SELF)

    # Calculate instantaneous RTF from delta since last sample
    wall_delta = wall_elapsed - _perf.last_sample_wall_time
    sim_delta = _perf.sim_time - _perf.last_sample_sim_time

    if wall_delta > 0 and sim_delta > 0:
        rtf = sim_delta / wall_delta
    else:
        rtf = 0.0

    # Snapshot current function timing data (deep copy)
    func_timing_snapshot = {
        name: {"total_time": data["total_time"], "call_count": data["call_count"]}
        for name, data in _perf.timing_data.items()
    }

    sample = TimeSample(
        wall_time=wall_elapsed,
        sim_time=_perf.sim_time,
        rtf=rtf,
        cpu_user=cpu.ru_utime - _perf.start_cpu.ru_utime,
        cpu_system=cpu.ru_stime - _perf.start_cpu.ru_stime,
        total_calls=sum(d["call_count"] for d in _perf.timing_data.values()),
        func_timing=func_timing_snapshot,
    )
    _perf.samples.append(sample)

    # Update last sample tracking
    _perf.last_sample_wall_time = wall_elapsed
    _perf.last_sample_sim_time = _perf.sim_time


def _write_results() -> None:
    """Write timing results to files on exit."""
    if not _perf.timing_data:
        return

    # Collect final sample
    _collect_sample()

    _write_summary()
    _write_csv()


def _write_summary() -> None:
    """Write human-readable summary file."""
    output_path = f"{_perf.output_prefix}.txt"

    # Calculate final metrics
    elapsed_time = time.time() - _perf.start_time
    end_cpu = resource.getrusage(resource.RUSAGE_SELF)
    user_cpu_time = end_cpu.ru_utime - _perf.start_cpu.ru_utime
    system_cpu_time = end_cpu.ru_stime - _perf.start_cpu.ru_stime
    total_cpu_time = user_cpu_time + system_cpu_time
    cpu_usage_percent = (total_cpu_time / elapsed_time) * 100 if elapsed_time > 0 else 0

    # Calculate overall RTF from total simulation time vs wall clock time
    overall_rtf = _perf.sim_time / elapsed_time if elapsed_time > 0 else 0.0

    # Calculate average RTF from samples (for comparison)
    rtf_values = [s.rtf for s in _perf.samples if s.rtf > 0]
    avg_sample_rtf = sum(rtf_values) / len(rtf_values) if rtf_values else 0.0

    # Sort functions by average time (descending)
    sorted_data = sorted(
        _perf.timing_data.items(),
        key=lambda x: x[1]["total_time"] / max(x[1]["call_count"], 1),
        reverse=True,
    )

    lines = [
        f"=== {_perf.simulator_name} Performance Report ===",
        f"Wall clock time: {elapsed_time:.2f} s",
        f"Simulation time: {_perf.sim_time:.2f} s",
        f"Overall RTF: {overall_rtf:.2f}",
        f"Average sample RTF: {avg_sample_rtf:.2f}",
        f"CPU time (user): {user_cpu_time:.2f} s",
        f"CPU time (system): {system_cpu_time:.2f} s",
        f"CPU time (total): {total_cpu_time:.2f} s",
        f"CPU usage: {cpu_usage_percent:.1f}%",
        f"Samples collected: {len(_perf.samples)}",
        "",
        "=== Function Timing ===",
    ]

    for func_name, data in sorted_data:
        if data["call_count"] > 0:
            avg_time_us = (data["total_time"] / data["call_count"]) * 1e6
            total_time_ms = data["total_time"] * 1e3
            lines.append(
                f"{func_name}: {avg_time_us:.2f} µs avg, {total_time_ms:.1f} ms total, {data['call_count']} calls"
            )

    output = "\n".join(lines) + "\n"

    # Write to file
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(output)

    #  print(f"\n=== Performance timing results written to {output_path} ===")
    #  print(output)


def _write_csv() -> None:
    """Write time-series data to CSV for graphing."""
    output_path = f"{_perf.output_prefix}.csv"

    if not _perf.samples:
        return

    # Get all function names
    func_names = sorted(_perf.timing_data.keys())

    # CSV header
    header = [
        "wall_time",
        "sim_time",
        "rtf",
        "cpu_user",
        "cpu_system",
        "cpu_total",
        "total_calls",
    ]
    # Add per-function columns
    for name in func_names:
        header.extend([f"{name}_avg_us", f"{name}_calls"])

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        # Write each sample
        for sample in _perf.samples:
            row = [
                f"{sample.wall_time:.3f}",
                f"{sample.sim_time:.3f}",
                f"{sample.rtf:.3f}",
                f"{sample.cpu_user:.3f}",
                f"{sample.cpu_system:.3f}",
                f"{sample.cpu_user + sample.cpu_system:.3f}",
                sample.total_calls,
            ]

            # Add per-function data from the snapshot at this sample point
            for name in func_names:
                data = sample.func_timing.get(name, {"total_time": 0.0, "call_count": 0})
                if data["call_count"] > 0:
                    avg_us = (data["total_time"] / data["call_count"]) * 1e6
                    row.extend([f"{avg_us:.2f}", data["call_count"]])
                else:
                    row.extend(["0.00", "0"])

            writer.writerow(row)

    #  print(f"Time-series data written to {output_path}")


# Register the write function to run at exit and on signals
atexit.register(_write_results)


def _signal_handler(signum, frame):
    """Handle termination signals by writing results before exit."""
    _write_results()
    # Re-raise the signal to allow normal termination
    signal.signal(signum, signal.SIG_DFL)
    signal.raise_signal(signum)


# Register signal handlers for graceful shutdown
signal.signal(signal.SIGTERM, _signal_handler)
signal.signal(signal.SIGINT, _signal_handler)


def timed(func: F) -> F:
    """
    Decorator that measures execution time of a function.

    Results are accumulated and written to files at program exit.

    Usage:
        @timed
        def my_function():
            ...
    """
    func_name = func.__name__

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        result = func(*args, **kwargs)
        elapsed = time.perf_counter() - start

        _perf.timing_data[func_name]["total_time"] += elapsed
        _perf.timing_data[func_name]["call_count"] += 1

        # Collect sample and write results periodically
        total_calls = sum(d["call_count"] for d in _perf.timing_data.values())
        if total_calls - _perf.last_write_count >= _perf.sample_interval:
            _perf.last_write_count = total_calls
            _collect_sample()
            _write_results()

        return result

    return wrapper  # type: ignore


def timed_method(name: str | None = None) -> Callable[[F], F]:
    """
    Decorator factory for timing methods with a custom name.

    Usage:
        @timed_method("publish_imu")
        def publish_imu_event(self):
            ...
    """

    def decorator(func: F) -> F:
        func_name = name or func.__name__

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            elapsed = time.perf_counter() - start

            _perf.timing_data[func_name]["total_time"] += elapsed
            _perf.timing_data[func_name]["call_count"] += 1

            return result

        return wrapper  # type: ignore

    return decorator


def get_timing_summary() -> str:
    """Get the current timing summary as a string."""
    if not _perf.timing_data:
        return "No timing data collected."

    sorted_data = sorted(
        _perf.timing_data.items(),
        key=lambda x: x[1]["total_time"] / max(x[1]["call_count"], 1),
        reverse=True,
    )

    lines = []
    for func_name, data in sorted_data:
        if data["call_count"] > 0:
            avg_time_us = (data["total_time"] / data["call_count"]) * 1e6
            lines.append(f"Elapsed time for {func_name}: {avg_time_us:.2f} µs over {data['call_count']} calls")

    return "\n".join(lines)


def reset_timing() -> None:
    """Reset all timing data."""
    _perf.timing_data.clear()
    _perf.samples.clear()
    _perf.start_time = time.time()
    _perf.start_cpu = resource.getrusage(resource.RUSAGE_SELF)
    _perf.last_write_count = 0
    _perf.sim_time = 0.0
    _perf.last_sample_wall_time = 0.0
    _perf.last_sample_sim_time = 0.0
