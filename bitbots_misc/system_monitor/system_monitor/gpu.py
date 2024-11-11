import pyamdgpuinfo


def collect_all():
    if pyamdgpuinfo.detect_gpus() == 0:
        return 0

    gpu = pyamdgpuinfo.get_gpu(0)
    vram_size = gpu.VRAM  # change VRAM to the real attribute name
    vram_usage = gpu.query_vram_usage()

    return vram_usage / vram_size
