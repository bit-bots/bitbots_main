# Use upstream ubuntu images as base
FROM nvidia/cuda:10.2-cudnn7-devel
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

# Install a lot of apt packages. They would also be installed with rosdep, but we want them to be cached
RUN apt-get install -y build-essential git sudo python3-pip python3-rospkg python3-catkin-pkg \
    python3-catkin-lint python3-rosdep ros-melodic-ros-base locales wget python3-catkin-tools \
    ros-melodic-urdf ros-melodic-tf2 ros-melodic-tf2-sensor-msgs ros-melodic-tf-conversions \
    python3-opencv ros-melodic-gazebo-msgs ros-melodic-imu-complementary-filter xvfb ros-melodic-ros-numpy \
    ros-melodic-map-msgs ros-melodic-move-base ros-melodic-spatio-temporal-voxel-layer ros-melodic-moveit-core \
    ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-planning-interface ros-melodic-robot-state-publisher \
    python3-sphinx-rtd-theme ros-melodic-image-transport ros-melodic-eigen-conversions python-hypothesis \
    python3-protobuf espeak ros-melodic-xacro ros-melodic-cv-bridge ros-melodic-moveit-ros-robot-interaction \
    ros-melodic-control-toolbox libprotobuf-dev protobuf-compiler libprotoc-dev ros-melodic-map-server \
    python3-psutil python3-hypothesis

# Set up locale
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen && locale-gen && update-locale LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Add user
ARG UID=150
RUN useradd -M -d /catkin_ws -s /bin/bash -u $UID robot

# Install sudoers file
ADD sudoers /etc/sudoers

WORKDIR /catkin_ws
RUN chown robot:robot /catkin_ws
USER robot:robot
ENV PATH=$PATH:/catkin_ws/.local/bin

RUN . /opt/ros/melodic/setup.sh && \
    mkdir src && \
    catkin init && \
    catkin config --profile default --extend /opt/ros/melodic -DPYTHON_VERSION=3 -DCMAKE_BUILD_TYPE=Release
# Add some requirements already here so that they are cached
RUN python3 -m pip install -U pip && \
    pip3 install -U PyYAML construct defusedxml filterpy matplotlib numpy opencv-python \
    protobuf psutil pytorchyolo rosdep rospkg setuptools sklearn transforms3d

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
    wolfgang_pybullet_sim bitbots_navigation/bitbots_visual_compass lib/DynamixelSDK lib/dynamixel-workbench \
    lib/dynamixel_workbench_msgs bitbots_misc/bitbots_live_tool_rqt bitbots_motion/bitbots_recordui_rqt && \
    sed -i '/plotjuggler/d' bitbots_motion/bitbots_quintic_walk/package.xml && \
    sed -i '/run_depend/d' wolfgang_robot/wolfgang_moveit_config/package.xml
RUN sudo rosdep init && rosdep update
RUN rosdep install -iry --from-paths src && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Delete some tests (has to happen before catkin build)
RUN cd src/bitbots_meta && \
    rm bitbots_tools/bitbots_test/test/rostests/test_webots_simulator.launch \
       bitbots_navigation/bitbots_localization/test/rostests/test_inital_localization_side.launch \
       bitbots_motion/bitbots_quintic_walk/test/rostests/test_walk.launch

RUN catkin build

# Execute tests
RUN . devel/setup.sh && catkin run_tests && catkin_test_results

RUN cd src/bitbots_meta && make vision-files
RUN cp src/bitbots_meta/wolfgang_robot/wolfgang_robocup_api/scripts/start.sh .local/bin/start
# Set respawn="true" for all nodes
RUN bash -c "find -name '*.launch' | xargs sed -i '/<node/s/ respawn=\"\w\+\"//;/<node/s/ required=\"\w\+\"//;/<node/s/<node/<node respawn=\"true\"/'"

# Volume for logs
VOLUME /robocup-logs

# Setup runtime
ENV DEBIAN_FRONTEND=readline
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
