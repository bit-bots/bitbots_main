import pyamdgpuinfo


def collect_all():
    if pyamdgpuinfo.detect_gpus() == 0:
        return 0

    gpu = pyamdgpuinfo.get_gpu(0)
    load = gpu.query_load()
    vram_total = gpu.memory_info["vram_size"]
    vram_used = gpu.query_vram_usage()
    temperature = int(gpu.query_temperature())

    return (load, vram_used, vram_total, temperature)
