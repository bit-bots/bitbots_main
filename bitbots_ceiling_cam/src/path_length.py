#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import numpy as np
rospy.init_node("path_length")


def path_cb(path_msg):
    path_length = 0
    for i in range(len(path_msg.poses) - 1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y

        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))

    position_a_x = path_msg.poses[0].pose.position.x
    position_b_x = path_msg.poses[-1].pose.position.x
    position_a_y = path_msg.poses[0].pose.position.y
    position_b_y = path_msg.poses[-1].pose.position.y

    start_to_fin = np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))

    print(f"Path length: {path_length:.4f}, Start to finish length: {start_to_fin:.4f}")



rospy.Subscriber("/robot_path", Path, path_cb)
rospy.spin()