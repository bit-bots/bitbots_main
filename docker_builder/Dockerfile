# Use upstream melodic images as base
FROM ros:melodic-ros-base-bionic AS bitbots-builder

# Install system dependencies
RUN apt-get update && \
    apt-get install -y sudo python3-pip python-pip python-coverage python-xmlrunner python-rospkg python-catkin-pkg \
    python-catkin-lint python-rosdep ros-melodic-rosdoc-lite dia && \
    python3 -m pip install rospkg catkin-pkg

# Install sudoers file
ADD sudoers /etc/sudoers

# Add user
RUN useradd -M -d /catkin_ws -s /bin/bash -u 1001 builder

# Setup catkin workspace
RUN . /opt/ros/melodic/setup.sh && \
    mkdir -p /catkin_ws/src; cd /catkin_ws && \
    catkin init && \
    catkin build && \
	chown -R builder:builder /catkin_ws && \
    chmod -R 740 /catkin_ws

# Setup entrypoint
USER $user:$group
COPY entrypoint.bash /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
