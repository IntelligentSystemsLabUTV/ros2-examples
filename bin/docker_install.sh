#!/usr/bin/env bash

# Installation script for Docker Engine, Compose and additional tools.
#
# Roberto Masocco <robmasocco@gmail.com>
# Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
#
# December 14, 2022

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
  echo >&2 "Usage:"
  echo >&2 "    docker_install.sh"
  echo >&2 "It is interactive, so it must be executed, not sourced"
  exit 1
fi

# shellcheck source=/dev/null
DISTRIBUTION_ID=$(
  . /etc/os-release
  echo "$ID"
)

# Routine to install Nvidia runtime
function nvidia_runtime {
  echo "Setting up the stable repository..."
  local distribution
  # shellcheck source=/dev/null
  distribution=$(
    . /etc/os-release
    echo "$ID$VERSION_ID"
  ) &&
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg &&
    curl -s -L https://nvidia.github.io/libnvidia-container/"$distribution"/libnvidia-container.list |
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  echo "Installing the Nvidia runtime..."
  sudo apt-get update && sudo apt-get install -y nvidia-docker2
  echo "Restarting the Docker daemon..."
  sudo systemctl restart docker
}

# Purge preexisting (i.e. not forward-compatible) Docker installations
echo "Purging old installations..."
sudo apt-get remove -y docker docker-engine docker.io containerd runc || true
if [[ -d /var/lib/docker ]]; then
  sudo rm -rf /var/lib/docker
fi
if [[ -d /var/lib/containerd ]]; then
  sudo rm -rf /var/lib/containerd
fi

# Install some dependencies for apt
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
  ca-certificates \
  curl \
  gnupg \
  lsb-release

# Add Docker's official GPG key and stable repository
echo "Setting up Docker's official repository..."
curl -fsSL https://download.docker.com/linux/"$DISTRIBUTION_ID"/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/$DISTRIBUTION_ID $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
sudo apt-get update

# Install Docker Engine
echo "Installing Docker Engine..."
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

# Install Docker Compose V1 or V2
while true; do
  read -r -p "Do you wish to install Docker Compose V2? " yn
  case $yn in
  [Yy]*)
    # Install Compose V2
    echo "Installing Compose V2 and Compose Switch..."
    sudo mkdir -p /usr/local/lib/docker/cli-plugins
    sudo curl -SL "https://github.com/docker/compose/releases/download/v2.13.0/docker-compose-linux-$(uname -m)" -o /usr/local/lib/docker/cli-plugins/docker-compose
    sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
    sudo curl -fL "https://github.com/docker/compose-switch/releases/download/v1.0.5/docker-compose-linux-$(dpkg --print-architecture)" -o /usr/local/bin/compose-switch
    sudo chmod +x /usr/local/bin/compose-switch
    sudo ln -s /usr/local/bin/compose-switch /usr/local/bin/docker-compose
    break
    ;;
  [Nn]*)
    # Install Compose V1
    echo "Installing Compose V1..."
    sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    break
    ;;
  *)
    echo "Please answer yes or no"
    ;;
  esac
done

# Install QEMU to enable buildx
while true; do
  read -r -p "Do you wish to install QEMU for cross-platform image builds? " yn
  case $yn in
  [Yy]*)
    echo "Installing QEMU..."
    sudo apt-get install -y qemu-user-static
    break
    ;;
  [Nn]*)
    break
    ;;
  *)
    echo "Please answer yes or no"
    ;;
  esac
done

# Create docker group and add user to it
echo "Creating new group for Docker users and adding user $USER to it..."
sudo groupadd docker || true
sudo usermod -aG docker "$USER" || true
echo "You need to log off and on again to see this change!"

# Check if the user wants to install latest Nvidia runtime
while true; do
  read -r -p "Do you wish to install the Nvidia runtime? (Requires Nvidia drivers >= 418.81) " yn
  case $yn in
  [Yy]*)
    nvidia_runtime
    break
    ;;
  [Nn]*)
    break
    ;;
  *)
    echo "Please answer yes or no"
    ;;
  esac
done

