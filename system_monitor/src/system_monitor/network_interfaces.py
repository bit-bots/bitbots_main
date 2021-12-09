from collections import defaultdict

import rospy
from system_monitor.msg import NetworkInterface as NetworkInterfaceMsg


_prev_msgs = defaultdict(NetworkInterfaceMsg)
_prev_msg_time = rospy.Time(0)


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
    global _prev_msgs
    global _prev_msg_time

    now = rospy.Time.now()
    for interface, i_msg in msgs.items():
        i_msg.rate_received_bytes = int((i_msg.received_bytes - _prev_msgs[interface].received_bytes) * (now - _prev_msg_time).to_sec())
        i_msg.rate_received_packets = int((i_msg.received_packets - _prev_msgs[interface].received_packets) * (now - _prev_msg_time).to_sec())
        i_msg.rate_received_packets_errors = int((i_msg.received_packets_errors - _prev_msgs[interface].received_packets_errors) * (now - _prev_msg_time).to_sec())
        i_msg.rate_received_packets_dropped = int((i_msg.received_packets_dropped - _prev_msgs[interface].received_packets_dropped) * (now - _prev_msg_time).to_sec())
        i_msg.rate_sent_bytes = int((i_msg.sent_bytes - _prev_msgs[interface].sent_bytes) * (now - _prev_msg_time).to_sec())
        i_msg.rate_sent_packets = int((i_msg.sent_packets - _prev_msgs[interface].sent_packets) * (now - _prev_msg_time).to_sec())
        i_msg.rate_sent_packets_errors = int((i_msg.sent_packets_errors - _prev_msgs[interface].sent_packets_errors) * (now - _prev_msg_time).to_sec())
        i_msg.rate_sent_packets_dropped = int((i_msg.sent_packets_dropped - _prev_msgs[interface].sent_packets_dropped) * (now - _prev_msg_time).to_sec())
        i_msg.rate_sent_packets_collisions = int((i_msg.sent_packets_collisions - _prev_msgs[interface].sent_packets_collisions) * (now - _prev_msg_time).to_sec())

    _prev_msg_time = now
    _prev_msgs = msgs
