# Use upstream ubuntu images as base
FROM ubuntu:18.04 AS bitbots-builder
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic
ENV ROS_PYTHON_VERSION=3

# workaround for a bug during installation https://stackoverflow.com/a/25267015
RUN ln -s -f /bin/true /usr/bin/chfn

# Install system dependencies
RUN apt-get update
RUN apt-get install -y gnupg2
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 4C4EDF893374591687621C75C2F8DBB6A37B2874
RUN sh -c 'echo "deb [arch=amd64] http://packages.bit-bots.de bionic main" > /etc/apt/sources.list.d/ros.list'
RUN apt-get update
RUN apt-get install -y build-essential git sudo dia python3-pip python3-coverage python3-rospkg python3-catkin-pkg \
    python3-catkin-lint python3-sphinx python3-sphinx-rtd-theme python3-breathe ros-melodic-ros-base
RUN python3 -m pip install --force-reinstall exhale
RUN python3 -m pip install --force-reinstall git+https://github.com/catkin/catkin_tools
RUN python3 -m pip install --force-reinstall --upgrade git+https://github.com/bit-bots/rosdep.git

# Setup libraries in dedicated workspace and install it into /opt/ros/melodic
WORKDIR /lib_ws
RUN . /opt/ros/melodic/setup.sh && \
    mkdir src && \
    catkin init && \
    catkin config --profile default --extend /opt/ros/melodic --install --merge-install --install-space /opt/ros/melodic -DPYTHON_VERSION=3
RUN git clone --depth 1 https://github.com/TAMS-Group/bio_ik.git src/bio_ik
RUN git clone --depth 1 https://github.com/TAMS-Group/bio_ik_service.git src/bio_ik_service
RUN git clone --depth 1 https://github.com/bit-bots/DynamixelSDK.git src/dynamixel_sdk
RUN git clone --depth 1 https://github.com/bit-bots/dynamixel-workbench.git src/dynamixel_workbench
RUN git clone --depth 1 https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git src/dynamixel_workbench_msgs
RUN git clone --depth 1 https://github.com/bit-bots/rot_conv_lib.git src/rot_conv_lib
RUN rosdep init && rosdep update
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
RUN . /opt/ros/melodic/setup.sh && \
    mkdir -p $BITBOTS_CATKIN_WORKSPACE/src; cd /catkin_ws && \
    catkin init && \
    catkin config --profile default --extend /opt/ros/melodic && \
    catkin build && \
    chown -R builder:builder /catkin_ws && \
    chmod -R 740 /catkin_ws

# Setup rosdep
ADD rosdep.source.list /etc/ros/rosdep/sources.list.d/30-bitbots.list
USER builder:builder
RUN rosdep update

# Setup runtime
ENV DEBIAN_FRONTEND=readline
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
