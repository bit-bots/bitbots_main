#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node

from bitbots_msgs.msg import Workload as WorkloadMsg
from system_monitor import cpus, memory, network_interfaces
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import time


def main():
    rclpy.init(args=None)
    node = Node('system_monitor')
    pub = node.create_publisher(WorkloadMsg, 'system_workload', 1)
    diagnostic_pub = node.create_publisher(DiagnosticArray, 'diagnostics', 1)

    hostname = socket.gethostname()

    diag_array = DiagnosticArray()
    diag_cpu = DiagnosticStatus()
    # start all names with "SYSTEM" for diagnostic analysesr
    diag_cpu.name = "SYSTEMCPU"
    diag_cpu.hardware_id = "CPU"
    diag_mem = DiagnosticStatus()
    diag_mem.name = "SYSTEMMemory"
    diag_cpu.hardware_id = "Memory"

    node.declare_parameter('update_frequency', 10)
    node.declare_parameter('do_memory', True)
    node.declare_parameter('do_cpu', True)
    node.declare_parameter('cpu_load_percentage', 80)
    node.declare_parameter('memoroy_load_percentage', 80)
    node.declare_parameter('network_rate_received_errors', 10)
    node.declare_parameter('network_rate_send_errors', 10)

    rate = node.get_parameter('update_frequency').get_parameter_value().double_value
    do_memory = node.get_parameter('do_memory').get_parameter_value().bool_value
    do_cpu = node.get_parameter('do_cpu').get_parameter_value().bool_value
    cpu_load_percentage = node.get_parameter('cpu_load_percentage').get_parameter_value().double_value
    memoroy_load_percentage = node.get_parameter('memoroy_load_percentage').get_parameter_value().double_value
    network_rate_received_errors = node.get_parameter(
        'network_rate_received_errors').get_parameter_value().double_value
    network_rate_send_errors = node.get_parameter('network_rate_send_errors').get_parameter_value().double_value

    while rclpy.ok():
        last_send_time = time.time()
        running_processes, cpu_usages, overall_usage_percentage = cpus.collect_all(node.get_clock()) if do_cpu else (
            -1, [], 0)
        memory_available, memory_used, memory_total = memory.collect_all(node.get_clock()) if do_memory else (-1, -1, -1)
        interfaces = network_interfaces.collect_all(node.get_clock())

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
        if overall_usage_percentage >= cpu_load_percentage:
            diag_cpu.level = DiagnosticStatus.WARN
        else:
            diag_cpu.level = DiagnosticStatus.OK
        diag_array.status.append(diag_cpu)

        memory_usage = round((memory_used / memory_total) * 100, 2)
        diag_mem.message = str(memory_usage) + "%"
        if memory_usage >= memoroy_load_percentage:
            diag_mem.level = DiagnosticStatus.WARN
        else:
            diag_mem.level = DiagnosticStatus.OK
        diag_array.status.append(diag_mem)

        for interface in interfaces:
            diag_net = DiagnosticStatus()
            diag_net.name = "SYSTEM" + interface.name
            diag_net.hardware_id = interface.name
            if interface.rate_received_packets_errors >= network_rate_received_errors \
                    or interface.rate_sent_packets_errors >= network_rate_send_errors:
                diag_net.level = DiagnosticStatus.WARN
            else:
                diag_net.level = DiagnosticStatus.OK
            diag_array.status.append(diag_net)
        diag_array.header.stamp = node.get_clock().now()
        diagnostic_pub.publish(diag_array)

        # sleep to have correct rate. we dont use ROS time since we are interested in system things
        dt = time.time() - last_send_time
        time.sleep(max(0, (1 / rate) - dt))
