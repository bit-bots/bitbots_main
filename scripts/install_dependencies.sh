#!/bin/bash

# Check if ROS repositories are already set up
if [[ ! -d /etc/apt/sources.list.d/ros-final.list ]]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-final.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
fi

sudo apt update
sudo apt install -y \
    ros-kinetic-desktop-full \
    \
    python \
    python-catkin-tools \
    python-pip \
    python3 \
    python3-pip \
    ros-kinetic-control-msgs \
    ros-kinetic-controller-manager \
    ros-kinetic-effort-controllers \
    ros-kinetic-gazebo-dev \
    ros-kinetic-gazebo-msgs \
    ros-kinetic-gazebo-ros \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-imu-complementary-filter \
    ros-kinetic-imu-sensor-controller \
    ros-kinetic-joint-state-controller \
    ros-kinetic-joint-trajectory-controller \
    ros-kinetic-joy \
    ros-kinetic-moveit-ros-control-interface \
    ros-kinetic-moveit-ros-move-group \
    ros-kinetic-moveit-ros-planning \
    ros-kinetic-moveit-ros-planning-interface \
    ros-kinetic-moveit-ros-robot-interaction \
    ros-kinetic-moveit-simple-controller-manager \
    ros-kinetic-navigation \
    ros-kinetic-pointcloud-to-laserscan \
    ros-kinetic-position-controllers \
    ros-kinetic-robot-controllers \
    ros-kinetic-robot-localization \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-rosbridge-server \
    ros-kinetic-rosdoc-lite \
    ros-kinetic-rqt-controller-manager \
    ros-kinetic-velocity-controllers \
    ros-kinetic-yocs-velocity-smoother \
    vim \
    screen\

# Python3 dependencies
DIR="$(dirname $(dirname $(realpath $0)))"
echo -ne "\nI will use pip3 to install some python3 dependencies. You may want to activate a virtualenvironment.\nContinue? (y/n)"
read CONTINUE
if [[ $CONTINUE != "n" ]] ; then
    pip3 install --user -r "${DIR}/requirements3.txt"
fi


# Python2 dependencies
echo -ne "\nI will use pip2 to install some python2 dependencies. You may want to activate a virtualenvironment.\nContinue? (y/n)"
read CONTINUE
if [[ $CONTINUE != "n" ]] ; then
    pip2 install --user -r "${DIR}/requirements2.txt"
fi


# Gazebo world models
echo -e "\nCopying models to gazebo (simulator)"
mkdir -p ~/.gazebo
cp -r ${DIR}/humanoid_league_visualization/humanoid_league_gazebo_world/models/ ~/.gazebo/


echo -e "\ndone"

