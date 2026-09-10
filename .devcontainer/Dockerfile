FROM ubuntu:24.04

ARG user=bitbots
ARG uid=1000
ARG gid=1000

# Basic Utilities
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
RUN apt-get update -y \
  && apt-get install -y \
    apt-utils \
    build-essential \
    clang-format \
    curl \
    ffmpeg \
    fzf \
    gdb \
    git \
    gnupg2 \
    htop \
    iproute2 \
    iptables \
    iputils-ping \
    jq \
    less \
    libncurses5-dev \
    locales \
    netcat-openbsd \
    openssh-server \
    python3-pip \
    rsync \
    screen \
    sudo \
    tmux \
    tree \
    vim \
    wget \
    x11-apps \
    zsh \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Setup locale
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen \
  && locale-gen \
  && update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# SSH configuration
RUN mkdir -p /var/run/sshd \
  && sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin no/' /etc/ssh/sshd_config \
  && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication no/' /etc/ssh/sshd_config

# Create bitbots user
RUN if getent passwd "$uid" >/dev/null; then \
      userdel -rf $(getent passwd "$uid" | cut -d: -f1); \
    fi \
  && if getent group "$gid" >/dev/null; then \
      groupdel $(getent group "$gid" | cut -d: -f1); \
    fi \
  && groupadd -g "$gid" "$user" \
  && useradd -m -u "$uid" -g "$gid" -G sudo -s /bin/zsh "$user" \
  && echo "$user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Install pixi
USER $user
RUN curl -fsSL https://pixi.sh/install.sh | sh

# Setup dotfiles and zimfw (matching ansible setup)
USER root
RUN mkdir -p /usr/share/zimfw \
  && curl -fsSL https://github.com/zimfw/zimfw/releases/latest/download/zimfw.zsh -o /usr/share/zimfw/zimfw.zsh \
  && chmod +r /usr/share/zimfw/zimfw.zsh

# Copy dotfiles from ansible_files
COPY --chown=$user:$user docker/ansible_files/zshrc /home/${user}/.zshrc
COPY --chown=$user:$user docker/ansible_files/zshenv /home/${user}/.zshenv
COPY --chown=$user:$user docker/ansible_files/zlogin /home/${user}/.zlogin
COPY --chown=$user:$user docker/ansible_files/zimrc /home/${user}/.zimrc
COPY --chown=$user:$user docker/ansible_files/vimrc /home/${user}/.vimrc
COPY --chown=$user:$user docker/ansible_files/tmux.conf /home/${user}/.tmux.conf
COPY --chown=$user:$user docker/ansible_files/screenrc /home/${user}/.screenrc
RUN mkdir -p /home/${user}/.zim /home/${user}/.config/htop && chown -R $user:$user /home/${user}/.zim /home/${user}/.config
COPY --chown=$user:$user docker/ansible_files/htoprc /home/${user}/.config/htop/htoprc

# Install zim modules
USER $user
RUN zsh /usr/share/zimfw/zimfw.zsh install

# Setup SSH key for bitbots user
ARG ssh_pub_key
RUN mkdir -p /home/${user}/.ssh \
  && if [ -n "$ssh_pub_key" ]; then echo "${ssh_pub_key}" > /home/${user}/.ssh/authorized_keys; fi \
  && chown -R ${user}:${user} /home/${user}/.ssh \
  && chmod 700 /home/${user}/.ssh \
  && if [ -f /home/${user}/.ssh/authorized_keys ]; then chmod 600 /home/${user}/.ssh/authorized_keys; fi

# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1

# Switch to the workspace directory
USER $user
WORKDIR /home/$user/bitbots_main
