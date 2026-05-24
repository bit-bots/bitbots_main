from rclpy.node import Node

# Detect available GPU backend (deferred until we have a node for logging)
_gpu_backend = None


def _detect_gpu_backend(node: Node):
    """Auto-detect available GPU and return appropriate backend function."""
    global _gpu_backend

    # Try NVIDIA first (most common in robotics)
    try:
        import pynvml

        # nvmlInit can fail if the NVIDIA driver/ NVML library is not installed
        try:
            pynvml.nvmlInit()
        except Exception as e:
            node.get_logger().debug(f"pynvml present but nvmlInit failed: {e}")
        else:
            try:
                device_count = pynvml.nvmlDeviceGetCount()
                if device_count > 0:
                    _gpu_backend = _collect_nvidia
                    node.get_logger().info(f"Detected NVIDIA GPU (pynvml): {device_count} device(s)")
                    return
            except Exception as e:
                node.get_logger().debug(f"NVIDIA GPU detection failed: {e}")
            finally:
                try:
                    pynvml.nvmlShutdown()
                except Exception:
                    pass
    except ImportError:
        node.get_logger().debug("pynvml not available")

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
        import pynvml

        pynvml.nvmlInit()
        try:
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            load = float(pynvml.nvmlDeviceGetUtilizationRates(handle).gpu)
            mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
            vram_used = mem_info.used
            vram_total = mem_info.total
            temperature = float(pynvml.nvmlDeviceGetTemperature(handle, 0))
            return (load, vram_used, vram_total, temperature)
        finally:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass
    except Exception as e:
        node.get_logger().error(f"Error collecting NVIDIA GPU metrics: {e}")
        return (0.0, 0, 0, 0.0)


def _collect_amd(node: Node) -> tuple[float, int, int, float]:
    """Collect GPU metrics from AMD GPU using pyamdgpuinfo."""
    try:
        import pyamdgpuinfo

        if pyamdgpuinfo.detect_gpus() == 0:
            return (0.0, 0, 0, 0.0)

        gpu = pyamdgpuinfo.get_gpu(0)
        load = float(gpu.query_load())
        vram_total = gpu.memory_info["vram_size"]
        vram_used = gpu.query_vram_usage()
        temperature = float(gpu.query_temperature())

        return (load, vram_used, vram_total, temperature)
    except Exception as e:
        node.get_logger().error(f"Error collecting AMD GPU metrics: {e}")
        return (0.0, 0, 0, 0.0)


def collect_all(node: Node) -> tuple[float, int, int, float]:
    """Collect GPU metrics using the auto-detected backend.

    If `node` is provided the ROS node's logger will be used for messages.

    node: ROS node for logging (required for backend detection and error logging)
    :return: (load, vram_used, vram_total, temperature)
    """
    if _gpu_backend is None:
        _detect_gpu_backend(node)
    if _gpu_backend is None:
        return (0.0, 0, 0, 0.0)
    return _gpu_backend(node)
