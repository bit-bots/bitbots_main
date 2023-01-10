# Use upstream ubuntu images as base
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update
RUN apt install -y wget
RUN mkdir -p /usr/local/share/keyrings
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/local/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=amd64 signed-by=/usr/local/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list
RUN wget https://packages.bit-bots.de/key.asc -O /usr/local/share/keyrings/bitbots.key
RUN echo "deb [signed-by=/usr/local/share/keyrings/bitbots.key arch=amd64] https://packages.bit-bots.de jammy main" | tee /etc/apt/sources.list.d/bitbots.list
RUN apt-get update

# Install a lot of apt packages. They could also be installed with rosdep, but we want them to be cached
RUN apt-get install -y build-essential git sudo python3-pip python3-rospkg \
    ros-rolling-ros-base locales wget python3-colcon-core python3-colcon-common-extensions \
    ros-rolling-urdf ros-rolling-tf2 ros-rolling-tf2-sensor-msgs \
    python3-opencv ros-rolling-gazebo-msgs ros-rolling-imu-complementary-filter xvfb \
    ros-rolling-map-msgs ros-rolling-navigation2 ros-rolling-moveit-core \
    ros-rolling-moveit-ros-planning ros-rolling-moveit-ros-planning-interface ros-rolling-robot-state-publisher \
    python3-sphinx-rtd-theme ros-rolling-image-transport ros-rolling-vision-msgs ros-rolling-test-msgs \
    python3-protobuf espeak ros-rolling-xacro ros-rolling-cv-bridge ros-rolling-moveit-ros-robot-interaction \
    ros-rolling-control-toolbox libprotobuf-dev protobuf-compiler libprotoc-dev \
    python3-psutil python3-hypothesis ros-rolling-ament-cmake-nose ros-rolling-rot-conv \
    ros-rolling-camera-info-manager ros-rolling-image-geometry ros-rolling-soccer-vision-3d-msgs

# Set up locale
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen && locale-gen && update-locale LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Add user
ARG UID=150
RUN useradd -M -d /colcon_ws -s /bin/bash -u $UID robot

# Install sudoers file
ADD sudoers /etc/sudoers

WORKDIR /colcon_ws
RUN chown robot:robot /colcon_ws
USER robot:robot
ENV PATH=$PATH:/colcon_ws/.local/bin

RUN . /opt/ros/rolling/setup.sh && \
    mkdir src

# Add some requirements already here so that they are cached
RUN python3 -m pip install -U pip && \
    pip3 install -U PyYAML construct defusedxml filterpy matplotlib numpy opencv-python \
    protobuf psutil pytorchyolo setuptools sklearn transforms3d

# From here on, we don't want to cache anything. That's achieved by adding the current time.
ADD https://www.timeapi.io/api/Time/current/zone?timeZone=UTC /tmp/build-time

RUN cd src && \
    git clone --recursive https://github.com/Bit-Bots/bitbots_meta.git && \
    cd bitbots_meta && \
    make pull-init && \
    sed -i -e /pydot/d -e /silx/d -e /pyopencl/d requirements.txt && \
    pip3 install -U -r requirements.txt && \
    rm -rf ~/.cache
# Make image size smaller: remove unused packages or unused dependencies
RUN cd src/bitbots_meta && \
    rm -rf bitbots_tools/bitbots_jenkins_library udp_bridge bitbots_tools/containers \
    humanoid_league_visualization dynamic_stack_decider/dynamic_stack_decider_visualization bitbots_lowlevel \
    wolfgang_robot/wolfgang_pybullet_sim bitbots_navigation/bitbots_visual_compass lib/DynamixelSDK lib/dynamixel-workbench \
    lib/events-executor/events_executor_examples lib/events-executor/irobot_lock_free_events_queue \
    bitbots_misc/bitbots_live_tool_rqt  bitbots_motion/bitbots_recordui_rqt bitbots_misc/bitbots_basler_camera \
    lib/pylon-ros-camera bitbots_misc/bitbots_extrinsic_calibration && \
    sed -i '/\bmoveit_ros\b/d' bitbots_motion/*/CMakeLists.txt && \
    sed -i '/plotjuggler/d' bitbots_motion/bitbots_quintic_walk/package.xml && \
    sed -i '/run_depend/d' wolfgang_robot/wolfgang_moveit_config/package.xml

RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Delete some tests (has to happen before colcon build)
RUN cd src/bitbots_meta && \
    rm bitbots_tools/bitbots_test/test/rostests/test_webots_simulator.launch \
       bitbots_navigation/bitbots_localization/test/rostests/test_inital_localization_side.launch \
       bitbots_motion/bitbots_quintic_walk/test/rostests/test_walk.launch

RUN . /opt/ros/rolling/setup.sh && colcon build

# TODO execute tests

RUN cd src/bitbots_meta && make vision-files
RUN cp src/bitbots_meta/wolfgang_robot/wolfgang_robocup_api/scripts/start.sh .local/bin/start

# Volume for logs
VOLUME /robocup-logs

# Setup runtime
ENV DEBIAN_FRONTEND=readline
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
