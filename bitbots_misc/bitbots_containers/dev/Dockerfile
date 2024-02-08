FROM ros:iron

# Arguments
ARG user
ARG uid
ARG home
ARG workspace
ARG shell

# Basic Utilities
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y \
  && apt-get install -y apt-utils auto-apt-proxy \
  && apt-get install -y \
    build-essential \
    curl \
    gdb \
    gnupg2 \
    htop \
    iproute2 \
    iputils-ping \
    ipython3 \
    jq \
    less \
    libncurses5-dev \
    locales \
    ranger \
    screen \
    ssh \
    sudo \
    synaptic \
    tig \
    tmux \
    tree \
    uvcdynctrl \
    vim \
    vlc \
    wget \
    x11-apps \
    zsh

# Setup locale
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen \
  && locale-gen \
  && update-locale LANG=en_US.UTF-8 \
  && ln -s /usr/bin/python3 /usr/bin/python

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Setup and prioritize packages.bit-bots.de repository
RUN apt update -y \
  && apt upgrade -y --allow-downgrades

# Additional custom dependencies
RUN apt-get install -y \
  espeak \
  ffmpeg \
  libespeak-dev \
  libfmt-dev \
  librange-v3-dev \
  librostest-dev \
  libtf-conversions-dev \
  liburdfdom-dev \
  libyaml-cpp-dev \
  llvm \
  protobuf-compiler \
  python3-colcon-common-extensions \
  python3-colcon-ed \
  python3-construct \
  python3-pip \
  python3-protobuf \
  python3-pybind11 \
  python3-rosdep \
  radeontop \
  ros-iron-camera-calibration \
  ros-iron-camera-info-manager \
  ros-iron-controller-manager \
  ros-iron-desktop \
  ros-iron-diagnostic-aggregator \
  ros-iron-effort-controllers \
  ros-iron-gazebo-msgs \
  ros-iron-image-proc \
  ros-iron-joint-state-broadcaster \
  ros-iron-joint-state-publisher-gui \
  ros-iron-joint-trajectory-controller \
  ros-iron-joy-linux \
  ros-iron-moveit-planners-ompl \
  ros-iron-moveit-ros \
  ros-iron-moveit-simple-controller-manager \
  ros-iron-nav2-bringup \
  # ros-iron-plotjuggler-ros containing plotjuggler ros plugins
  # build currently fails and is not available as a package so we
  # have to manually install plotjuggler and plotjuggler-msgs
  # https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/59
  ros-iron-plotjuggler \
  ros-iron-plotjuggler-msgs \
  ros-iron-position-controllers \
  ros-iron-rmw-cyclonedds-cpp \
  ros-iron-robot-localization \
  ros-iron-rot-conv \
  ros-iron-rqt-robot-monitor \
  ros-iron-soccer-vision-2d-msgs \
  ros-iron-soccer-vision-3d-rviz-markers \
  ros-iron-test-msgs \
  ros-iron-tf-transformations \
  ros-iron-transmission-interface \
  ros-iron-velocity-controllers \
  && pip3 install pip -U \
  && python3 -m pip install git+https://github.com/ruffsl/colcon-clean

# Mount the user's home directory
VOLUME "${home}"

# Clone user into docker image and set up X11 sharing
RUN echo "${user}:x:${uid}:${uid}:${user},,,:${home}:${shell}" >> /etc/passwd \
  && echo "${user}:*::0:99999:0:::" >> /etc/shadow \
  && echo "${user}:x:${uid}:" >> /etc/group \
  && echo "${user} ALL=(ALL) NOPASSWD: ALL" >> "/etc/sudoers"

# Switch to user
USER "${user}"
# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1
# Switch to the workspace
WORKDIR ${workspace}
