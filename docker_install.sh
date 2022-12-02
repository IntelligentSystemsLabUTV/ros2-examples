#!/bin/bash

# Docker Engine and Docker Compose installation script (updated as of today).
#
# Roberto Masocco <robmasocco@gmail.com>
# Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
#
# January 26, 2022

# Routine to install Nvidia runtime
function nvidia_runtime {
  echo "Setting up the stable repository..."
  distribution=$(. /etc/os-release; echo $ID$VERSION_ID) \
    && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
    && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   echo "Installing the Nvidia runtime..."
   sudo apt-get update && sudo apt-get install -y nvidia-docker2
   echo "Restarting the Docker daemon..."
   sudo systemctl restart docker
}

# Purge preexisting (i.e. not forward-compatible) Docker installations
echo "Purging old installations..."
sudo apt-get remove -y docker docker-engine docker.io containerd runc
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
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install Docker Engine
echo "Installing Docker Engine..."
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

# Install Docker Compose V2
echo "Installing Compose V2 and Compose Switch..."
sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo curl -SL "https://github.com/docker/compose/releases/download/v2.13.0/docker-compose-linux-$(uname -s)" -o /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
sudo curl -fL "https://github.com/docker/compose-switch/releases/download/v1.0.5/docker-compose-linux-$(dpkg --print-architecture)" -o /usr/local/bin/compose-switch
sudo chmod +x /usr/local/bin/compose-switch
sudo ln -s /usr/local/bin/compose-switch /usr/local/bin/docker-compose

# Create docker group and add user to it
echo "Creating new group for Docker users and adding $USER to it..."
sudo groupadd docker
sudo usermod -aG docker "$USER"
echo "You need to log off and on again to see this change!"

# Check if the user wants to install latest Nvidia runtime
while true; do
  read -r -p "Do you wish to install the Nvidia runtime? (Requires Nvidia drivers >= 418.81) " yn
  case $yn in
    [Yy]* ) nvidia_runtime; break;;
    [Nn]* ) break;;
    * ) echo "Please answer yes or no";;
  esac
done
