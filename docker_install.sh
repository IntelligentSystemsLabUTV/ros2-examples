#!/bin/bash

# Docker Engine and Docker Compose installation script (updated as of today).
#
# Roberto Masocco <robmasocco@gmail.com>
# Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
#
# January 26, 2022

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

# Install the current stable release of Docker Compose
echo "Installing Docker Compose..."
sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
