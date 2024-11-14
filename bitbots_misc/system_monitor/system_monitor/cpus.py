from collections import defaultdict

import psutil

from bitbots_msgs.msg import Cpu as CpuMsg

_prev_total = defaultdict(int)
_prev_busy = defaultdict(int)


def collect_all():
    """
    parse /proc/stat and calculate total and busy time

    (more specific USER_HZ see man 5 proc for further information )
    """
    msgs = []
    running_processes = len(psutil.pids())
    cpu_timings = _get_cpu_stats()
    overall_usage = 0

    for cpu, timings in cpu_timings.items():
        cpu_total = sum(timings)
        del timings[3:5]
        cpu_busy = sum(timings)
        cpu_usage = _calculate_usage(cpu, cpu_total, cpu_busy)

        msgs.append(CpuMsg(cpu_name=cpu, cpu_usage=cpu_usage))
        overall_usage += cpu_usage

    # compute mean of cpu usages
    overall_usage_percentage = overall_usage / len(cpu_timings)
    return running_processes, msgs, overall_usage_percentage


def _get_cpu_stats():
    """
    read and parse /proc/stat

    :returns: timings which contains accumulative busy and total cpu time
    """
    timings = {}
    with open("/proc/stat") as file_obj:
        for line in file_obj:
            # only evaluate lines like cpu0, cpu1, cpu2, ...
            if line.startswith("cpu") and line.strip().split()[0] != "cpu":
                line = line.strip().split()
                timings[line[0]] = [int(x) for x in line[1:]]

    return timings


def _calculate_usage(cpu_num, total, busy):
    """
    calculate usage percentage based on busy/total time(load, vram_used, vram_total, temperature)
    """
    diff_total = total - _prev_total[cpu_num]
    diff_busy = busy - _prev_busy[cpu_num]

    _prev_total[cpu_num] = total
    _prev_busy[cpu_num] = busy

    if diff_total == 0:
        return 0
    else:
        return float(int(diff_busy / diff_total * 100))
