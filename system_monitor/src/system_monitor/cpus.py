from collections import defaultdict

from system_monitor.msg import Cpu as CpuMsg

_prev_total = defaultdict(int)
_prev_busy = defaultdict(int)


def collect_all():
    """
    parse /proc/stat and calculate total and busy time

    (more specific USER_HZ see man 5 proc for further information )
    """
    msgs = []
    timings, running_processes = _get_cpu_stats()

    for cpu, timings in timings.items():
        cpu_total = sum(timings)
        del timings[3:5]
        cpu_busy = sum(timings)
        cpu_usage = _calculate_usage(cpu, cpu_total, cpu_busy)

        msgs.append(CpuMsg(
            cpu_name=cpu,
            cpu_usage=cpu_usage
        ))

    return running_processes, msgs


def _get_cpu_stats():
    """
    read and parse /proc/stat
    :returns Tuple[timings, open_processes]
    """
    timings = {}
    processes = -1
    with open('/proc/stat', 'r') as file_obj:
        for line in file_obj:
            # only evaluate lines like cpu0, cpu1, cpu2, ...
            if line.startswith('cpu') and line.strip().split()[0] != 'cpu':
                line = line.strip().split()
                timings[line[0]] = [int(x) for x in line[1:]]

            elif line.startswith('procs_running'):
                processes = int(line.strip().split()[1])

    return timings, processes


def _calculate_usage( cpu, total, busy):
    """
    calculate usage
    """
    diff_total = total - _prev_total[cpu]
    diff_busy = busy - _prev_busy[cpu]

    _prev_total[cpu] = total
    _prev_busy[cpu] = busy

    if diff_total == 0:
        return 0
    else:
        return int(diff_busy / diff_total * 100)
