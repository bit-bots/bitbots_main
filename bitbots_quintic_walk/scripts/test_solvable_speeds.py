#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
from bitbots_moveit_bindings import set_moveit_parameters
from bitbots_utils.utils import load_moveit_parameter, get_parameters_from_ros_yaml
from numpy import random
from deep_quintic.engine import WalkEngine
from geometry_msgs.msg import Twist

def cmd_vel_to_twist(cmd_vel, stop=False):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = float(cmd_vel[0])
    cmd_vel_msg.linear.y = float(cmd_vel[1])
    cmd_vel_msg.linear.z = 0.0
    cmd_vel_msg.angular.x = 0.0
    cmd_vel_msg.angular.y = 0.0
    cmd_vel_msg.angular.z = float(cmd_vel[2])
    if stop:
        cmd_vel_msg.angular.x = -1.0
    return cmd_vel_msg

sim_name = "webots"
robot_type = "wolfgang"
cmd_vel_current_bounds = [(-0.5, 0.5), (-0.2, 0.2), (-2, 2)]
experiment_number = 1000
threshold = 0.0001

walk_parameters = get_parameters_from_ros_yaml("walking",
                                                f"{get_package_share_directory('bitbots_quintic_walk')}/config/deep_quintic_{sim_name}_{robot_type}.yaml",
                                                use_wildcard=True)
moveit_parameters = load_moveit_parameter(robot_type)                                                
engine = WalkEngine("", walk_parameters + moveit_parameters)

okay = 0
not_okay = 0
print(f"\nRandom sampling {experiment_number} velocities for robot {robot_type} for {sim_name} parameters.")
print(f"Velocity space is {cmd_vel_current_bounds}")
print(f"Threshold is {threshold}")
try:
    while True:
        current_command_speed = [random.uniform(*cmd_vel_current_bounds[0]),
                                random.uniform(*cmd_vel_current_bounds[1]),
                                random.uniform(*cmd_vel_current_bounds[2])]
                                                                        
        if engine.reset_and_test_if_speed_possible(cmd_vel_to_twist(current_command_speed), threshold):
            #print(f"{current_command_speed} SOLVED")
            okay += 1
        else:
            #print(f"{current_command_speed}")
            not_okay += 1
        
        if okay + not_okay > experiment_number:
            fraction = okay/(okay+not_okay)
            print(f"Solvable {fraction}")
            break

except:
    fraction = okay/(okay+not_okay)
    print(f"Solvable {fraction}")
