# Use upstream ubuntu images as base
FROM nvidia/cuda:12.8.0-devel-ubuntu24.04

ARG UID=150
ARG ROS_DISTRO=jazzy
ARG BITBOTS_MAIN_BRANCH=feature/jazzy-ubuntu2404-devcontainer

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update \
    && apt install -y wget auto-apt-proxy apt-utils \
    && mkdir -p /usr/local/share/keyrings \
    && wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/local/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/local/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list \
    && apt update

# Install a lot of apt packages.
# They could also be installed with rosdep, but we want them to be cached.
RUN apt install -y \
        build-essential \
        espeak \
        ffmpeg \
        git \
        ipython3 \
        locales \
        protobuf-compiler \
        sudo \
        vcstool \
        gir1.2-gst-plugins-base-1.0 \
        gir1.2-gstreamer-1.0 \
        gstreamer1.0-alsa \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-tools \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libfreeimage-dev\
        libgstreamer-plugins-base1.0-0 \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-0 \
        libgstreamer1.0-dev \
        libjsoncpp-dev \
        libopencv-dev \
        libprotobuf-dev \
        libprotoc-dev \
        librange-v3-dev \
        liburdfdom-dev \
        libyaml-cpp-dev \
        protobuf-compiler \
        pybind11-dev \
        pyqt5-dev \
        python3-breathe \
        python3-colcon-clean \
        python3-colcon-common-extensions \
        python3-construct \
        python3-coverage \
        python3-cryptography \
        python3-hypothesis \
        python3-matplotlib \
        python3-nose \
        python3-numpy \
        python3-opencv \
        python3-pil \
        python3-pip \
        python3-protobuf \
        python3-psutil \
        python3-pydot \
        python3-pyqt5 \
        python3-pyqt5.qtsvg \
        python3-pytest \
        python3-rosdep \
        python3-rospkg \
        python3-scipy \
        python3-sip-dev \
        python3-sphinx-rtd-theme \
        python3-tk \
        python3-transforms3d \
        python3-yaml \
        xvfb \
        ros-${ROS_DISTRO}-action-msgs \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-cmake-mypy \
        ros-${ROS_DISTRO}-ament-cmake-python \
        ros-${ROS_DISTRO}-ament-cmake-ros \
        ros-${ROS_DISTRO}-ament-copyright \
        ros-${ROS_DISTRO}-ament-flake8 \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
        ros-${ROS_DISTRO}-ament-mypy \
        ros-${ROS_DISTRO}-ament-pep257 \
        ros-${ROS_DISTRO}-apriltag-ros \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-controller-interface \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-diagnostic-aggregator \
        ros-${ROS_DISTRO}-diagnostic-msgs \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-gazebo-msgs \
        ros-${ROS_DISTRO}-generate-parameter-library \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-image-proc \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-imu-complementary-filter \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-launch-xml \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-moveit-core \
        ros-${ROS_DISTRO}-moveit-kinematics \
        ros-${ROS_DISTRO}-moveit-msgs \
        ros-${ROS_DISTRO}-moveit-planners-ompl \
        ros-${ROS_DISTRO}-moveit-ros \
        ros-${ROS_DISTRO}-moveit-ros-move-group \
        ros-${ROS_DISTRO}-moveit-ros-planning \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface \
        ros-${ROS_DISTRO}-moveit-ros-robot-interaction \
        ros-${ROS_DISTRO}-moveit-ros-visualization \
        ros-${ROS_DISTRO}-moveit-setup-assistant \
        ros-${ROS_DISTRO}-moveit-simple-controller-manager \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-plotjuggler \
        ros-${ROS_DISTRO}-plotjuggler \
        ros-${ROS_DISTRO}-pluginlib \
        ros-${ROS_DISTRO}-pybind11-vendor \
        ros-${ROS_DISTRO}-qt-dotgraph \
        ros-${ROS_DISTRO}-rcl \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-rclcpp-components \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros2launch \
        ros-${ROS_DISTRO}-ros2trace \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-${ROS_DISTRO}-rosgraph-msgs \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        ros-${ROS_DISTRO}-rot-conv \
        ros-${ROS_DISTRO}-rqt-gui \
        ros-${ROS_DISTRO}-rqt-gui-py \
        ros-${ROS_DISTRO}-rviz-imu-plugin \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-sensor-msgs-py \
        ros-${ROS_DISTRO}-shape-msgs \
        ros-${ROS_DISTRO}-soccer-vision-2d-msgs \
        ros-${ROS_DISTRO}-soccer-vision-3d-msgs \
        ros-${ROS_DISTRO}-soccer-vision-3d-rviz-markers \
        ros-${ROS_DISTRO}-soccer-vision-attribute-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-test-msgs \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-tf2 \
        ros-${ROS_DISTRO}-tf2-eigen \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-tf2-kdl \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-sensor-msgs \
        ros-${ROS_DISTRO}-topic-tools \
        ros-${ROS_DISTRO}-tracetools-acceleration \
        ros-${ROS_DISTRO}-tracetools-analysis \
        ros-${ROS_DISTRO}-tracetools-image-pipeline \
        ros-${ROS_DISTRO}-tracetools-test \
        ros-${ROS_DISTRO}-trajectory-msgs \
        ros-${ROS_DISTRO}-transmission-interface \
        ros-${ROS_DISTRO}-urdf \
        ros-${ROS_DISTRO}-urdfdom-py \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-xacro \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen \
    && locale-gen \
    && update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN mkdir -p /ws \
    && useradd -M -d /ws -s /bin/bash -u $UID robot \
    && chown -R robot:robot /ws \
    && echo "robot ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/robot

WORKDIR /ws
USER robot:robot
ENV PATH=$PATH:/ws/.local/bin

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && sudo rosdep init

RUN mkdir -p -m 700 $HOME/.ssh \
    && ssh-keyscan github.com >> $HOME/.ssh/known_hosts

RUN --mount=type=ssh,uid=$UID git clone --branch "$BITBOTS_MAIN_BRANCH" git@github.com:bit-bots/bitbots_main.git  \
    && cd bitbots_main \
    && just pull-init rosdep pip

WORKDIR /ws/bitbots_main

# From here on, we don't want to cache anything. That's achieved by adding the current time.
ADD https://www.timeapi.io/api/Time/current/zone?timeZone=UTC /tmp/build-time

RUN --mount=type=ssh,uid=$UID \
    && just pull-init \
    && cd src \
    && rm -rf bitbots_lowlevel \
        bitbots_misc/bitbots_basler_camera \
        bitbots_misc/bitbots_ceiling_cam \
        bitbots_misc/bitbots_containers \
        bitbots_motion/bitbots_animation_rqt \
        bitbots_simulation/bitbots_pybullet_sim \
        bitbots_team_communication/bitbots_team_data_sim_rqt \
        bitbots_wolfgang \
        lib/dynamic_stack_decider/dynamic_stack_decider_visualization \
        lib/dynamixel-workbench \
        lib/pylon-ros-camera \
        lib/soccer_field_map_generator \
        lib/udp_bridge \
    && sed -i '/plotjuggler/d' bitbots_motion/bitbots_quintic_walk/package.xml

# Install missing ros dependencies with rosdep
RUN sudo apt update \
  && rosdep update \
  && rosdep install --rosdistro=${ROS_DISTRO} --from-paths src --ignore-src -r -y \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --cmake-args -DBUILD_TESTING=OFF

RUN cp src/bitbots_simulation/bitbots_robocup_api/scripts/start.sh $HOME/.local/bin/start

# Volume for logs
VOLUME /robocup-logs

# ROS bags get created in $HOME/rosbags
# Thats why we create a symlink to /robocup-logs
RUN ln -s /robocup-logs $HOME/rosbags

# Setup runtime
ENV DEBIAN_FRONTEND=readline
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
