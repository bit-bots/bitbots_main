#!/bin/bash
set -e

# Refreshes submodules
git submodule sync

# This script switches submodules to the ros2 development branch.
echo "./bitbots_behavior" && cd ./bitbots_behavior && git switch ros-2-migration ; cd -
echo "./bitbots_lowlevel" && cd ./bitbots_lowlevel && git switch ros-2-migration ; cd -
echo "./bitbots_misc" && cd ./bitbots_misc && git switch ros2 ; cd -
echo "./bitbots_motion" && cd ./bitbots_motion && git switch ros2 ; cd -
echo "./bitbots_msgs" && cd ./bitbots_msgs && git switch ros2-devel ; cd -
echo "./bitbots_navigation" && cd ./bitbots_navigation && git switch feature/ros-2-upgrade ; cd -
echo "./bitbots_tools" && cd ./bitbots_tools && git switch ros2 ; cd -
echo "./bitbots_vision" && cd ./bitbots_vision && git switch ros2-devel ; cd -
echo "./bitbots_world_model" && cd ./bitbots_world_model && git switch feature/ros2 ; cd -
echo "./bitbots_behavior" && cd ./bitbots_behavior && git switch ros-2-migration ; cd -
echo "./center_of_mass" && cd ./center_of_mass && git switch ros2 ; cd -
echo "./dynamic_stack_decider" && cd ./dynamic_stack_decider && git switch ros2 ; cd -
echo "./humanoid_base_footprint" && cd ./humanoid_base_footprint && git switch master ; cd -
echo "./humanoid_league_misc" && cd ./humanoid_league_misc && git switch feature/ros-2-upgrade ; cd -
echo "./humanoid_league_msgs" && cd ./humanoid_league_msgs && git switch feature/ros-2-upgrade ; cd -
echo "./humanoid_league_visualization" && cd ./humanoid_league_visualization && git switch migrate/ros2/interactive_marker ; cd -
echo "./udp_bridge" && cd ./udp_bridge && git switch ros2 ; cd -
echo "./wolfgang_robot" && cd ./wolfgang_robot && git switch ros2-devel ; cd -
echo "./lib/bio_ik" && cd ./lib/bio_ik && git switch ros2 ; cd -
echo "./lib/bio_ik_service" && cd ./lib/bio_ik_service && git switch ros2 ; cd -
echo "./lib/fp" && cd ./lib/fp && git switch main ; cd -
echo "./bitbots_tools" && cd ./bitbots_tools && git switch ros2 ; cd - 
echo "./lib/rot_conv_lib" && cd ./lib/rot_conv_lib && git switch ros2 ; cd -
echo "./lib/bio_ik" && cd ./lib/bio_ik && git switch ros2 ; cd -
echo "./lib/bio_ik_service" && cd ./lib/bio_ik_service && git switch ros2 ; cd -
echo "./humanoid_league_msgs" && cd ./humanoid_league_msgs && git switch feature/ros-2-upgrade ; cd -
echo "./lib/dynamixel-workbench" && cd ./lib/dynamixel-workbench && git switch ros2 ; cd -
echo "./lib/spatio_temporal_voxel_layer" && cd ./lib/spatio_temporal_voxel_layer && git switch ros2 ; cd -
echo "./lib/particle_filter" && cd ./lib/particle_filter && git switch ros2 ; cd -
