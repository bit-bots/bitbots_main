# Use upstream noetic images as base
FROM docker.io/ros:noetic-ros-base AS bitbots-builder

# Install system dependencies
RUN apt-get update
RUN apt-get remove -y ros-noetic-rosdoc-lite python3-sphinx
RUN apt-get install -y git sudo dia python3-pip python3-coverage python3-rospkg python3-catkin-pkg \
    python3-catkin-lint python3-rosdep python3-sphinx python3-sphinx-rtd-theme python3-breathe
RUN python3 -m pip install exhale
RUN python3 -m pip install git+https://github.com/catkin/catkin_tools

# Setup libraries in dedicated workspace and install it into /opt/ros/noetice
WORKDIR /lib_ws
RUN . /opt/ros/noetic/setup.sh && \
    mkdir src && \
    catkin init && \
    catkin config --profile default --extend /opt/ros/noetic --install --merge-install --install-space /opt/ros/noetic
RUN git clone --depth 1 https://github.com/TAMS-Group/bio_ik.git src/bio_ik
RUN git clone --depth 1 https://github.com/TAMS-Group/bio_ik_service.git src/bio_ik_service
RUN git clone --depth 1 https://github.com/bit-bots/DynamixelSDK.git src/dynamixel_sdk
RUN git clone --depth 1 https://github.com/bit-bots/dynamixel-workbench.git src/dynamixel_workbench
RUN git clone --depth 1 https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git src/dynamixel_workbench_msgs
RUN git clone --depth 1 https://github.com/bit-bots/rot_conv_lib.git src/rot_conf_lib
RUN git clone --depth 1 -b noetic-devel https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git src/spatio_temporal_voxel_layer
RUN rosdep install -iry --from-paths src
RUN catkin build

# Install sudoers file
ADD sudoers /etc/sudoers

# Add user
ARG UID=150
RUN useradd -M -d /catkin_ws -s /bin/bash -u $UID builder

# Setup catkin workspace
ENV BITBOTS_CATKIN_WORKSPACE=/catkin_ws
WORKDIR $BITBOTS_CATKIN_WORKSPACE
RUN . /opt/ros/noetic/setup.sh && \
    mkdir -p $BITBOTS_CATKIN_WORKSPACE/src; cd /catkin_ws && \
    catkin init && \
    catkin config --profile default --extend /opt/ros/noetic && \
    catkin build && \
    chown -R builder:builder /catkin_ws && \
    chmod -R 740 /catkin_ws

# Setup rosdep
ADD rosdep.source.list /etc/ros/rosdep/sources.list.d/30-bitbots.list
RUN su -c "rosdep update" builder

# Setup runtime
ENV DEBIAN_FRONTEND=readline
ENV ROS_PYTHON_VERSION=3
USER builder:builder
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
