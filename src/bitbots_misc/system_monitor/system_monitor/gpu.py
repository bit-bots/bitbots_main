import pyamdgpuinfo


def collect_all() -> tuple[float, int, int, float]:
    """
    use pyamdgpuinfo to get gpu metrics

    :return: (load, vram_used, vram_total, temperature)
    """
    if pyamdgpuinfo.detect_gpus() == 0:
        return (0.0, 0, 0, 0.0)

    gpu = pyamdgpuinfo.get_gpu(0)
    load = float(gpu.query_load())
    vram_total = gpu.memory_info["vram_size"]
    vram_used = gpu.query_vram_usage()
    temperature = float(gpu.query_temperature())

    return (load, vram_used, vram_total, temperature)
