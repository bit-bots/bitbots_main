cmake_minimum_required(VERSION 2.8.3)
project(bitbots_bezier_pathfinding)

find_package(catkin REQUIRED rospy geometry_msgs nav_msgs dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/pathfinding.cfg)

catkin_package()
