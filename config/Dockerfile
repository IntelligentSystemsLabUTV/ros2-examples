# Example development environment based on Ubuntu 20.04.
#
# Roberto Masocco <robmasocco@gmail.com>
# Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
#
# January 25, 2022

# RUN THIS WITH:
# --cap-add=SYS_PTRACE
# --security-opt seccomp=unconfined
# --network host
# --user ros
# -v ~/.ssh:/home/$USERNAME/.ssh:ro
# -v .aliases.zsh:/home/$USERNAME/.aliases.zsh
# -v .p10k.zsh:/home/$USERNAME/.p10k.zsh
# -v .ros2_cmds.zsh:/home/$USERNAME/.ros2_cmds.zsh
# -v .zshrc:/home/$USERNAME/.zshrc
# -v zsh_history:/home/$USERNAME/zsh_history
# If your terminal emulator supports colors also add:
# --env TERM=xterm-256color
# If you want to start GUI-based applications also add:
# --env="DISPLAY"
# If you use an Nvidia GPU and want to use the Nvidia runtime also add:
# --runtime=nvidia

### BASE UBUNTU 20.04 INSTALLATION ###
FROM ubuntu:20.04 as base

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

# Install basic utilities
RUN apt-get update && apt-get install -y \
  apt-utils \
  python3-pip \
  software-properties-common \
  curl \
  iproute2 && \
  add-apt-repository universe && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Install language and locales
RUN apt-get update && apt-get install -y \
  locales && \
  locale-gen en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*
ENV LANG en_US.UTF-8

# Set up UTC timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -y tzdata && \
  dpkg-reconfigure --frontend noninteractive tzdata && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Install ROS 2 base and testing packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  apt-get update && apt-get install -y \
  gnupg2 \
  lsb-release \
  sudo \
  ros-galactic-ros-base \
  ros-galactic-eigen3-cmake-module \
  ros-galactic-ament-lint \
  ros-galactic-launch-testing \
  ros-galactic-launch-testing-ament-cmake \
  ros-galactic-launch-testing-ros \
  python3-argcomplete \
  python3-colcon-argcomplete \
  python3-colcon-common-extensions \
  python3-rosdep \
  libasio-dev \
  libtinyxml2-dev && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*
RUN yes | pip3 install -U \
  empy \
  pyros-genmsg \
  setuptools \
  testresources

# Install development tools
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  gdb \
  git \
  pylint3 \
  python3-vcstool \
  python3-autopep8 \
  vim \
  nano \
  manpages-posix-dev \
  wget \
  valgrind \
  tree \
  xclip \
  minicom \
  neofetch \
  htop \
  screen && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Install Zsh
RUN apt-get update && apt-get install -y \
  zsh \
  zsh-doc \
  chroma && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Cleanup
RUN apt-get autoclean && \
  apt-get autoremove && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Create a non-root sudo user with Zsh as shell
RUN groupadd --gid $USER_GID $USERNAME && \
  useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
  echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME
ENV HOME=/home/$USERNAME
RUN chsh -s /usr/bin/zsh $USERNAME

# Switch to internal user
USER $USERNAME
WORKDIR $HOME

# Copy user configuration files
COPY --chown=$USER_UID:$USER_GID .nanorc ./
COPY --chown=$USER_UID:$USER_GID .vimrc ./

# Create user SSH directory
RUN mkdir .ssh

# Create workspace directory: host workspaces will be mounted here
RUN mkdir workspace

# Configure Zsh for internal user
ENV ZSH=$HOME/.oh-my-zsh
ARG ZSH_CUSTOM=$ZSH/custom
RUN wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | zsh || true
RUN \
  ZSH_PLUGINS=$ZSH_CUSTOM/plugins \
  && ZSH_THEMES=$ZSH_CUSTOM/themes \
  && git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
  && git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
  && git clone --single-branch --depth 1 https://github.com/romkatv/powerlevel10k.git $ZSH_THEMES/powerlevel10k
COPY --chown=$USER_UID:$USER_GID .zshrc ./
COPY --chown=$USER_UID:$USER_GID .ros2_cmds.zsh ./
COPY --chown=$USER_UID:$USER_GID .aliases.zsh ./
COPY --chown=$USER_UID:$USER_GID .p10k.zsh ./

# Set environment variables
ENV ROS_DISTRO=galactic
ENV AMENT_PREFIX_PATH=/opt/ros/galactic
ENV COLCON_PREFIX_PATH=/opt/ros/galactic
ENV LD_LIBRARY_PATH=/opt/ros/galactic/lib
ENV PATH=/opt/ros/galactic/bin:$PATH
ENV PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

ENV DEBIAN_FRONTEND=dialog

# Set default shell to start
CMD ["zsh"]

### ROS 2 DEVELOPMENT ENVIRONMENT ###
# NOTE: To use this, install nvidia-docker2 and run with nvidia runtime
FROM base as dev

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

USER root

# Install remaining ROS 2 packages
RUN apt-get update && apt-get install -y \
  ros-galactic-desktop \
  ros-galactic-gazebo-ros-pkgs && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

USER $USERNAME

### NVIDIA-compatible environment ###
FROM dev as nvidia

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

USER root

# Install Nvidia dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

USER $USERNAME

# Env vars for the nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM 1
