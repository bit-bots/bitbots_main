import psutil


def collect_all():
    """
    :return: (memory_available, memory_used, memory_total)
    """
    memory_usage = psutil.virtual_memory()
    return (memory_usage.available, memory_usage.used, memory_usage.total)
