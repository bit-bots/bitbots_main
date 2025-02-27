FROM ros:jazzy

ARG user=bitbots
ARG uid=1000
ARG gid=1000

# Basic Utilities
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y \
  && apt-get install -y apt-utils auto-apt-proxy \
  && apt-get install -y \
    build-essential \
    clang-format \
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
    rsync \
    screen \
    ssh \
    sudo \
    synaptic \
    tig \
    tmux \
    tree \
    uvcdynctrl \
    vim \
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
  python3-virtualenv \
  ros-jazzy-camera-calibration \
  ros-jazzy-desktop \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-plotjuggler \
  ros-jazzy-plotjuggler-msgs \
  ros-jazzy-plotjuggler-ros \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rqt-robot-monitor \
  ros-jazzy-soccer-vision-3d-rviz-markers

# Set zsh as default shell
SHELL ["/bin/zsh", "-c"]

# Remove the users group, because when it exists on the host system
# the devcontainer will not dynamically update the containerUser GID,
# when the host user is part of the users group.
# Then create a bitbots user with home directory and add allow it to use sudo
RUN groupdel users \
  && userdel -r ubuntu \
  && useradd -m -U -u "$uid" -G sudo -s /bin/zsh $user \
  && groupmod -g "$gid" $user \
  && echo "$user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER $user

# Install pip colcon extensions / patches as user
RUN python3 -m pip install --user --break-system-packages \
    git+https://github.com/ruffsl/colcon-clean \
    git+https://github.com/timonegk/colcon-core.git@colors \
    git+https://github.com/timonegk/colcon-notification.git@colors \
    git+https://github.com/timonegk/colcon-output.git@colors

# Create colcon workspace
RUN mkdir -p /home/$user/colcon_ws/src

# Install oh-my-zsh for pretty terminal
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
  git clone https://github.com/zsh-users/zsh-autosuggestions "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions"

# Add zshrc to bitbots home directory
COPY --chown=$user:$user zshrc /home/$user/.zshrc

# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1

# Switch to the workspace directory
WORKDIR /home/$user/colcon_ws
