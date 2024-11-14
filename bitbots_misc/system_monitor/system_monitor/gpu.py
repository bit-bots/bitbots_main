import pyamdgpuinfo


def collect_all():
    if pyamdgpuinfo.detect_gpus() == 0:
        return 0

    gpu = pyamdgpuinfo.get_gpu(0)
    vram_size = gpu.memory_info["vram_size"]
    vram_usage = gpu.query_vram_usage()
    load = gpu.query_load()
    temperature = gpu.query_temperature()

    return (vram_usage / vram_size, load, temperature)
