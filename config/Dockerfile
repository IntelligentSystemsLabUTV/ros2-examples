# ROS 2 development container.
#
# Roberto Masocco <robmasocco@gmail.com>
# Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
#
# January 25, 2022

# RUN THIS WITH:
# --cap-add=SYS_PTRACE
# --network host
# --user ros
# -v ~/.ssh:/home/ros/.ssh:ro
# -v .aliases.zsh:/home/ros/.aliases.zsh
# -v .p10k.zsh:/home/ros/.p10k.zsh
# -v .ros2_cmds.zsh:/home/ros/.ros2_cmds.zsh
# -v .zshrc:/home/ros/.zshrc
# -v zsh_history:/home/ros/zsh_history
# If your terminal emulator supports colors also add:
# --env TERM=xterm-256color
# If you want to start GUI-based applications also add:
# --env="DISPLAY"
# If you use an Nvidia GPU and want to use the Nvidia runtime also add:
# --runtime=nvidia

# Choose the container to build from
ARG TARGET=dev
FROM intelligentsystemslabutv/ros2-env:$TARGET

ARG USER_UID=1000
ARG USER_GID=1000

# Create a non-root sudo user with Zsh as shell
RUN useradd -s /bin/bash --uid $USER_UID -m ros && \
  echo ros ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/ros && \
  chmod 0440 /etc/sudoers.d/ros
ENV HOME=/home/ros
RUN chsh -s /usr/bin/zsh ros

# Create workspace directory: host workspaces will be mounted here
RUN mkdir $HOME/workspace && \
  chown ros $HOME/workspace

# Create shell history and SSH directory for user
RUN mkdir $HOME/zsh_history && \
  chown ros $HOME/zsh_history
RUN mkdir $HOME/.ssh

# Switch to internal user
USER ros
WORKDIR $HOME

# Copy user configuration files
COPY --chown=ros .nanorc ./
COPY --chown=ros .vimrc ./

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

# Add image information
LABEL author.name="Roberto Masocco"
LABEL author.email="robmasocco@gmail.com"
LABEL organization="Intelligent Systems Lab"
LABEL description="ROS 2 example development container"

# Start shell in container
CMD ["zsh"]
