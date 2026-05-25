import atexit
from pathlib import Path

from rclpy.node import Node

# Detect available GPU backend (deferred until we have a node for logging)
_gpu_backend = None
_nvml_module = None
_nvml_handle = None
_nvml_shutdown_registered = False

_JETSON_GPU_LOAD_PATHS = (
    Path("/sys/devices/gpu.0/load"),
    Path("/sys/kernel/debug/gpu.0/load"),
)
_JETSON_DEVICE_TREE_PATHS = (
    Path("/proc/device-tree/compatible"),
    Path("/proc/device-tree/model"),
)
_JETSON_GPU_TEMPERATURE_TYPES = ("gpu",)


def _detect_gpu_backend(node: Node):
    """Auto-detect available GPU and return appropriate backend function."""
    global _gpu_backend

    # Try NVIDIA first (most common in robotics)
    try:
        import pynvml
    except ImportError:
        node.get_logger().debug("pynvml not available")
    else:
        try:
            pynvml.nvmlInit()
            device_count = pynvml.nvmlDeviceGetCount()
            if device_count > 0:
                global _nvml_module, _nvml_handle, _nvml_shutdown_registered
                _nvml_module = pynvml
                _nvml_handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                if not _nvml_shutdown_registered:
                    atexit.register(_shutdown_nvml)
                    _nvml_shutdown_registered = True
                _gpu_backend = _collect_nvidia
                node.get_logger().info(f"Detected NVIDIA GPU (pynvml): {device_count} device(s)")
                return
        except Exception as e:
            node.get_logger().debug(f"NVIDIA GPU detection failed (pynvml): {e}")

    if _detect_jetson_gpu():
        _gpu_backend = _collect_jetson
        node.get_logger().info("Detected NVIDIA Jetson GPU (sysfs)")
        return

    # Try AMD next
    try:
        import pyamdgpuinfo

        if pyamdgpuinfo.detect_gpus() > 0:
            _gpu_backend = _collect_amd
            node.get_logger().info("Detected AMD GPU (pyamdgpuinfo)")
            return
    except ImportError:
        node.get_logger().debug("pyamdgpuinfo not available")
    except Exception as e:
        node.get_logger().debug(f"AMD GPU detection failed: {e}")

    # No GPU detected
    _gpu_backend = _collect_none
    node.get_logger().info("No GPU detected; falling back to null backend")


def _collect_none(node: Node) -> tuple[float, int, int, float]:
    """Null backend when no GPU is available."""
    return (0.0, 0, 0, 0.0)


def _collect_nvidia(node: Node) -> tuple[float, int, int, float]:
    """Collect GPU metrics from NVIDIA GPU using pynvml."""
    try:
        if _nvml_module is None or _nvml_handle is None:
            return (0.0, 0, 0, 0.0)

        load = _fraction_from_percent(float(_nvml_module.nvmlDeviceGetUtilizationRates(_nvml_handle).gpu))
        mem_info = _nvml_module.nvmlDeviceGetMemoryInfo(_nvml_handle)
        vram_used = mem_info.used
        vram_total = mem_info.total
        temperature = float(_nvml_module.nvmlDeviceGetTemperature(_nvml_handle, 0))
        return (load, vram_used, vram_total, temperature)
    except Exception as e:
        node.get_logger().error(f"Error collecting NVIDIA GPU metrics: {e}")
        return (0.0, 0, 0, 0.0)


def _detect_jetson_gpu() -> bool:
    """Detect NVIDIA Jetson GPUs, which often do not expose NVML devices."""
    if any(_path_exists(path) for path in _JETSON_GPU_LOAD_PATHS):
        return True

    for path in _JETSON_DEVICE_TREE_PATHS:
        try:
            content = path.read_bytes().lower()
        except OSError:
            continue
        if b"nvidia,tegra" in content or b"nvidia jetson" in content:
            return True

    return False


def _path_exists(path: Path) -> bool:
    try:
        return path.exists()
    except OSError:
        return False


def _read_float(path: Path) -> float | None:
    try:
        return float(path.read_text().strip())
    except (OSError, ValueError):
        return None


def _collect_jetson_temperature() -> float:
    for thermal_type_path in Path("/sys/devices/virtual/thermal").glob("thermal_zone*/type"):
        try:
            thermal_type = thermal_type_path.read_text().strip().lower()
        except OSError:
            continue

        if not any(gpu_type in thermal_type for gpu_type in _JETSON_GPU_TEMPERATURE_TYPES):
            continue

        temperature = _read_float(thermal_type_path.with_name("temp"))
        if temperature is None:
            continue
        return temperature / 1000.0 if temperature > 1000 else temperature

    return 0.0


def _collect_jetson(node: Node) -> tuple[float, int, int, float]:
    """Collect GPU metrics from NVIDIA Jetson sysfs files."""
    try:
        load = 0.0
        for path in _JETSON_GPU_LOAD_PATHS:
            raw_load = _read_float(path)
            if raw_load is None:
                continue
            # Jetson reports GPU load in permille on current L4T kernels.
            load = _fraction_from_per_mille(raw_load)
            break

        temperature = _collect_jetson_temperature()
        return (load, 0, 0, temperature)
    except Exception as e:
        node.get_logger().error(f"Error collecting NVIDIA Jetson GPU metrics: {e}")
        return (0.0, 0, 0, 0.0)


def _collect_amd(node: Node) -> tuple[float, int, int, float]:
    """Collect GPU metrics from AMD GPU using pyamdgpuinfo."""
    try:
        import pyamdgpuinfo

        if pyamdgpuinfo.detect_gpus() == 0:
            return (0.0, 0, 0, 0.0)

        gpu = pyamdgpuinfo.get_gpu(0)
        load = _fraction_from_percent(float(gpu.query_load()))
        vram_total = gpu.memory_info["vram_size"]
        vram_used = gpu.query_vram_usage()
        temperature = float(gpu.query_temperature())

        return (load, vram_used, vram_total, temperature)
    except Exception as e:
        node.get_logger().error(f"Error collecting AMD GPU metrics: {e}")
        return (0.0, 0, 0, 0.0)


def collect_all(node: Node) -> tuple[float, int, int, float]:
    """Collect GPU metrics using the auto-detected backend.

    node: ROS node for logging (required for backend detection and error logging)
    :return: (load, vram_used, vram_total, temperature)
    """
    if _gpu_backend is None:
        _detect_gpu_backend(node)
    if _gpu_backend is None:
        return (0.0, 0, 0, 0.0)
    return _gpu_backend(node)


def _fraction_from_percent(value: float) -> float:
    """Convert percent [0..100] to clamped fraction [0.0..1.0]."""
    return min(max(value / 100.0, 0.0), 1.0)


def _fraction_from_per_mille(value: float) -> float:
    """Convert permille [0..1000] to clamped fraction [0.0..1.0]."""
    return min(max(value / 1000.0, 0.0), 1.0)


def _shutdown_nvml() -> None:
    """Release NVML resources at process shutdown."""
    if _nvml_module is None:
        return
    try:
        _nvml_module.nvmlShutdown()
    except Exception:
        pass
