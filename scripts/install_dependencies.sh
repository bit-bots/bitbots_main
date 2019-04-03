#!/bin/bash

# Check if ROS repositories are already set up
if [[ ! -d /etc/apt/sources.list.d/ros-final.list ]]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-final.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
fi

sudo apt update
sudo apt install -y \
    ros-melodic-desktop-full \
    \
    dia \
    python \
    python-catkin-tools \
    python-pip \
    python-pyqtgraph \
    python3 \
    python3-pip \
    ros-melodic-control-msgs \
    ros-melodic-controller-manager \
    ros-melodic-effort-controllers \
    ros-melodic-gazebo-dev \
    ros-melodic-gazebo-msgs \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-control \
    ros-melodic-gazebo-plugins \
    ros-melodic-imu-complementary-filter \
    ros-melodic-imu-sensor-controller \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-joy \
    ros-melodic-moveit-ros-control-interface \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-ros-planning \
    ros-melodic-moveit-ros-planning-interface \
    ros-melodic-moveit-ros-robot-interaction \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-navigation \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-position-controllers \
    ros-melodic-robot-controllers \
    ros-melodic-robot-localization \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-rosbridge-server \
    ros-melodic-rosdoc-lite \
    ros-melodic-rqt-controller-manager \
    ros-melodic-velocity-controllers \
    ros-melodic-yocs-velocity-smoother \
    vim \
    screen \
    setserial \
    uvcdynctrl \

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

