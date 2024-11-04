import pyamdgpuinfo


def collect_all():
    gpus = pyamdgpuinfo.detect_gpus()

    if len(gpus) == 0:
        return
