# Use upstream ubuntu images as base
FROM ubuntu:18.04 AS bitbots
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
    python3-catkin-lint python3-rosdep python3-sphinx python3-sphinx-rtd-theme python3-breathe ros-melodic-ros-base \
    locales
RUN pip3 install -U pip git+https://github.com/catkin/catkin_tools.git

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

RUN . /opt/ros/melodic/setup.sh && \
    mkdir src && \
    catkin init && \
    catkin config --profile default --extend /opt/ros/melodic -DPYTHON_VERSION=3 -DCMAKE_BUILD_TYPE=Release
RUN cd src && \
    git clone --recursive https://github.com/Bit-Bots/bitbots_meta.git && \
    git -C bitbots_meta submodule foreach git checkout master && \
    python3 -m pip install -r bitbots_meta/requirements.txt
RUN sudo rosdep init && rosdep update
RUN rosdep install -iry --from-paths src
RUN catkin build

# Setup runtime
ENV DEBIAN_FRONTEND=readline
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
