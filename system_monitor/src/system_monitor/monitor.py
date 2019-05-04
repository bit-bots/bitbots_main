#!/usr/bin/env python3
import socket
import rospy

from system_monitor.msg import Workload as WorkloadMsg
from system_monitor import cpus, memory, network_interfaces


def validate_params():
    """@rtype bool"""
    def validate_single_param(param_name, required_type):
        """@rtype bool"""
        inner_result = True
        if not rospy.has_param(param_name):
            rospy.logfatal('Parameter {} is not defined but needed'.format(param_name))
            inner_result = False

        else:
            if type(required_type) is list and len(required_type) > 0:
                if type(rospy.get_param(param_name)) in required_type:
                    rospy.logfatal('Parameter {} is not any of type {}'.format(param_name, required_type))
                    inner_result = False
            else:
                if type(rospy.get_param(param_name)) is not required_type:
                    rospy.logfatal('Parameter {} is not of type {}'.format(param_name, required_type))
                    inner_result = False

        return inner_result

    result = True
    result = result and validate_single_param('/system_monitor/update_frequency', int)
    result = result and validate_single_param('/system_monitor/do_cpu', bool)
    result = result and validate_single_param('/system_monitor/do_memory', bool)
    result = result and validate_single_param('/system_monitor/do_network', bool)
    return result


if __name__ == '__main__':
    rospy.init_node('system_monitor', anonymous=True)
    if validate_params():
        pub = rospy.Publisher('/system_workload', WorkloadMsg, latch=True, queue_size=1)

        hostname = socket.gethostname()
        config = rospy.get_param('/system_monitor')

        while not rospy.is_shutdown():
            start_time = rospy.Time.now()

            running_processes, cpu_usages = cpus.collect_all() if config['do_cpu'] else (-1, [])
            memory_available, memory_used, memory_total = memory.collect_all() if config['do_memory'] else (-1, -1, -1)

            msg = WorkloadMsg(
                hostname=hostname,
                cpus=cpu_usages,
                running_processes=running_processes,
                memory_available=memory_available,
                memory_used=memory_used,
                memory_total=memory_total,
                filesystems=[],
                network_interfaces=network_interfaces.collect_all()
            )
            pub.publish(msg)

            end_time = rospy.Time.now()
            rospy.sleep(rospy.Duration(0, 1000000000 / config['update_frequency'] - end_time.nsecs + start_time.nsecs))  # 0.5 seconds
