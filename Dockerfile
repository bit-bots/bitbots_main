# Use upstream melodic images as base
FROM ros:melodic-ros-base-bionic AS bitbots-builder

ARG uid=1001
ARG gid=1001

# Install system dependencies
RUN apt-get update; \
    apt-get install -y python3-pip; \
    python3 -m pip install catkin-tools catkin-pkg trollius coverage unittest-xml-reporting

# Setup permissions for rosdep
RUN apt-get install -y sudo; \
    echo "Group = $gid"; groupadd -g $gid builder; \
    echo "User = $uid"; useradd -M -u $uid -g $gid builder; \
    echo "builder   ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Setup catkin workspace
RUN . /opt/ros/melodic/setup.sh; \
    mkdir -p /catkin_ws/src; cd /catkin_ws; \
    catkin init; \
    catkin config \
        -DPYTHON_EXECUTABLE=/usr/bin/python3.6 \
        -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
        -DPYTHON_LIBRARY=/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so; \
    catkin build; \
    chmod -R 777 /catkin_ws;

# Setup entrypoint
RUN echo "#!/bin/bash" > /ros_entrypoint.sh; \
    echo "set -e" >> /ros_entrypoint.sh; \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /ros_entrypoint.sh; \
    echo "source /catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh; \
    echo "exec \"\$@\"" >> /ros_entrypoint.sh;

# Add our scripts for convenience
#COPY scripts /opt/bitbots_scripts
