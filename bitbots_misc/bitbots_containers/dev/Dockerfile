FROM ros:iron

# Arguments
ARG user
ARG uid
ARG home

# Basic Utilities
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y \
  && apt-get install -y apt-utils auto-apt-proxy \
  && apt-get install -y \
    build-essential \
    curl \
    ffmpeg \
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
    python3-pip \
    radeontop \
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

# Additional robotics related packages
RUN apt-get install -y \
  python3-rosdep \
  python3-vcstool \
  ros-iron-camera-calibration \
  ros-iron-desktop \
  ros-iron-joint-state-publisher-gui \
  # ros-iron-plotjuggler-ros containing plotjuggler ros plugins
  # build currently fails and is not available as a package so we
  # have to manually install plotjuggler and plotjuggler-msgs
  # https://github.com/PlotJuggler/plotjuggler-ros-plugins/issues/59
  ros-iron-plotjuggler \
  ros-iron-plotjuggler-msgs \
  ros-iron-rmw-cyclonedds-cpp \
  ros-iron-rqt-robot-monitor \
  ros-iron-soccer-vision-3d-rviz-markers

# Update pip and install colcon-clean
RUN pip3 install pip -U

# Install colcon extensions / patches
RUN python3 -m pip install \
  git+https://github.com/ruffsl/colcon-clean \
  git+https://github.com/timonegk/colcon-core.git@colors \
  git+https://github.com/timonegk/colcon-notification.git@colors \
  git+https://github.com/timonegk/colcon-output.git@colors

# Clone user into docker image and set up X11 sharing
RUN echo "${user}:x:${uid}:${uid}:${user},,,:${home}:/usr/bin/zsh" >> /etc/passwd \
  && echo "${user}:*::0:99999:0:::" >> /etc/shadow \
  && echo "${user}:x:${uid}:" >> /etc/group \
  && echo "${user} ALL=(ALL) NOPASSWD: ALL" >> "/etc/sudoers"

# Create home directory and colcon workspace
RUN mkdir -p "${home}/colcon_ws" && chown -R "${user}:${user}" "${home}"

# Switch to user
USER "${user}"

# Install oh-my-zsh for pretty terminal
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
  git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions

# Add zshrc
COPY zshrc "${home}/.zshrc"

# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1

# Switch to the workspace directory
WORKDIR "${home}/colcon_ws"
