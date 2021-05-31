#!/usr/bin/env python3
import socket
import rospy

from system_monitor.msg import Workload as WorkloadMsg
from system_monitor import cpus, memory, network_interfaces
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import time


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
    result = result and validate_single_param('~update_frequency', int)
    result = result and validate_single_param('~do_cpu', bool)
    result = result and validate_single_param('~do_memory', bool)
    result = result and validate_single_param('~do_network', bool)
    return result


if __name__ == '__main__':
    rospy.init_node('system_monitor', anonymous=True)
    if validate_params():
        pub = rospy.Publisher('system_workload', WorkloadMsg, latch=True, queue_size=1)
        diagnostic_pub = rospy.Publisher('diagnostics', DiagnosticArray, latch=True, queue_size=1)

        hostname = socket.gethostname()
        config = rospy.get_param('~')
        rate = config['update_frequency']

        diag_array = DiagnosticArray()
        diag_cpu = DiagnosticStatus()
        # start all names with "SYSTEM" for diagnostic analysesr
        diag_cpu.name = "SYSTEMCPU"
        diag_cpu.hardware_id = "CPU"
        diag_mem = DiagnosticStatus()
        diag_mem.name = "SYSTEMMemory"
        diag_cpu.hardware_id = "Memory"

        while not rospy.is_shutdown():
            last_send_time = time.time()
            running_processes, cpu_usages, overall_usage_percentage = cpus.collect_all() if config['do_cpu'] else (
                -1, [], 0)
            memory_available, memory_used, memory_total = memory.collect_all() if config['do_memory'] else (-1, -1, -1)
            interfaces = network_interfaces.collect_all()

            msg = WorkloadMsg(
                hostname=hostname,
                cpus=cpu_usages,
                running_processes=running_processes,
                cpu_usage_overall=overall_usage_percentage,
                memory_available=memory_available,
                memory_used=memory_used,
                memory_total=memory_total,
                filesystems=[],
                network_interfaces=interfaces
            )
            pub.publish(msg)

            diag_array.status = []
            # publish diagnostic message to show this in the rqt diagnostic viewer
            diag_cpu.message = str(overall_usage_percentage) + "%"
            if overall_usage_percentage >= config['cpu_load_percentage']:
                diag_cpu.level = DiagnosticStatus.WARN
            else:
                diag_cpu.level = DiagnosticStatus.OK
            diag_array.status.append(diag_cpu)

            memory_usage = round((memory_used / memory_total) * 100, 2)
            diag_mem.message = str(memory_usage) + "%"
            if memory_usage >= config['memoroy_load_percentage']:
                diag_mem.level = DiagnosticStatus.WARN
            else:
                diag_mem.level = DiagnosticStatus.OK
            diag_array.status.append(diag_mem)

            for interface in interfaces:
                diag_net = DiagnosticStatus()
                diag_net.name = "SYSTEM" + interface.name
                diag_net.hardware_id = interface.name
                if interface.rate_received_packets_errors >= config['network_rate_received_errors'] \
                        or interface.rate_sent_packets_errors >= config['network_rate_send_errors']:
                    diag_net.level = DiagnosticStatus.WARN
                else:
                    diag_net.level = DiagnosticStatus.OK
                diag_array.status.append(diag_net)
            diag_array.header.stamp = rospy.Time.now()
            diagnostic_pub.publish(diag_array)

            # sleep to have correct rate. we dont use ROS time since we are interested in system things
            time.sleep(max(0, (1 / rate) - time.time() - last_send_time))
