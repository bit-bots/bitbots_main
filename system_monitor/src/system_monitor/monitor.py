#!/usr/bin/env python3
import socket
import rospy

from system_monitor import msg as SystemMonitorMsg
from system_monitor import cpus, memory


if __name__ == '__main__':
    rospy.init_node('system_monitor', anonymous=True)
    pub = rospy.Publisher('/system_workload', SystemMonitorMsg.Workload, latch=True, queue_size=1)
    hostname = socket.gethostname()

    while not rospy.is_shutdown():
        running_processes, cpu_usages = cpus.collect_all()
        memory_available, memory_used, memory_total = memory.collect_all()

        msg = SystemMonitorMsg.Workload(
            hostname=hostname,
            cpus=cpu_usages,
            running_processes=running_processes,
            memory_available=memory_available,
            memory_used=memory_used,
            memory_total=memory_total,
            filesystems=[],
            network_interfaces=[]
        )
        pub.publish(msg)

        rospy.sleep(rospy.Duration(0, 500000000))  # 0.5 seconds
