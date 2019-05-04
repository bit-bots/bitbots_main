import rospy
from system_monitor.msg import NetworkInterface as NetworkInterfaceMsg

_msg_cache = {}


def collect_all():
    msgs = _get_interfaces()
    _analyze_rate(msgs)
    return list(msgs.values())


def _get_interfaces():
    """@rtype: dict"""
    result = {}
    with open('/proc/net/dev') as file_obj:
        for line in file_obj:
            if '|' not in line:  # exclude table header lines
                line = line.strip().split()

                name = line[0].replace(':', '')
                msg = NetworkInterfaceMsg(
                    name=name,
                    received_bytes=int(line[1]),
                    received_packets=int(line[2]),
                    received_packets_errors=int(line[3]),
                    received_packets_dropped=int(line[4]),
                    sent_bytes=int(line[9]),
                    sent_packets=int(line[10]),
                    sent_packets_errors=int(line[11]),
                    sent_packets_dropped=int(line[12]),
                    sent_packets_collisions=int(line[14]),
                )
                result[name] = msg

    return result


def _analyze_rate(msgs):
    global _msg_cache

    # filter out cached messages older than 2 seconds
    now = rospy.Time.now()
    _msg_cache = {time: msgs for time, msgs in _msg_cache.items() if time > now - rospy.Duration(2)}
    _msg_cache[now] = msgs

    for interface, i_msg in msgs.items():
        i_msg.rate_received_bytes \
            = int(sum(i_cached_msgs[interface].received_bytes for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_received_packets \
            = int(sum(i_cached_msgs[interface].received_packets for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_received_packets_errors \
            = int(sum(i_cached_msgs[interface].received_packets_errors for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_received_packets_dropped \
            = int(sum(i_cached_msgs[interface].received_packets_dropped for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_sent_bytes \
            = int(sum(i_cached_msgs[interface].sent_bytes for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_sent_packets \
            = int(sum(i_cached_msgs[interface].sent_packets for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_sent_packets_errors \
            = int(sum(i_cached_msgs[interface].sent_packets_errors for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_sent_packets_dropped \
            = int(sum(i_cached_msgs[interface].sent_packets_dropped for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
        i_msg.rate_sent_packets_collisions \
            = int(sum(i_cached_msgs[interface].sent_packets_collisions for i_cached_msgs in _msg_cache.values())
                  / len(_msg_cache))
