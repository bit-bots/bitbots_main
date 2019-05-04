#!/usr/bin/env python3
import rospy

from system_monitor import msg as SystemMonitorMsg
from system_monitor import cpus


if __name__ == '__main__':
    rospy.init_node('system_monitor')
    pub = rospy.Publisher('/system_workload', SystemMonitorMsg.Workload, latch=True, queue_size=1)

    while not rospy.is_shutdown():
        num_processes, cpu_usages = cpus.collect_all()

        msg = SystemMonitorMsg.Workload(
            cpus=cpu_usages,
            memory_available=0,
            memory_used=0,
            num_processes=num_processes,
            filesystems=[],
            network_interfaces=[]
        )
        pub.publish(msg)

        rospy.sleep(rospy.Duration(0, 500000000))  # 0.5 seconds
