#!/bin/bash
set -e


# This script switches submodules to the ros2 development branch.
cd ./bitbots_behavior && git switch master ; cd -
cd ./bitbots_lowlevel && git switch master ; cd -
cd ./bitbots_misc && git switch ros2 ; cd -
cd ./bitbots_motion && git switch ros2 ; cd -
cd ./bitbots_msgs && git switch ros2-devel ; cd -
cd ./bitbots_navigation && git switch master ; cd -
cd ./bitbots_tools && git switch ros2 ; cd -
cd ./bitbots_vision && git switch ros2-devel ; cd -
cd ./bitbots_world_model && git switch feature/ros2 ; cd -
cd ./center_of_mass && git switch master ; cd -
cd ./dynamic_stack_decider && git switch master ; cd -
cd ./humanoid_base_footprint && git switch ros2 ; cd -
cd ./humanoid_league_misc && git switch feature/ros-2-upgrade ; cd -
cd ./humanoid_league_msgs && git switch feature/ros-2-upgrade ; cd -
cd ./humanoid_league_visualization && git switch migrate/ros2/interactive_marker ; cd -
cd ./udp_bridge && git switch master ; cd -
cd ./wolfgang_robot && git switch ros2-devel ; cd -
cd ./lib/bio_ik && git switch master ; cd -
cd ./lib/bio_ik_service && git switch ros2 ; cd -
