#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "=== Starting Docker Installation for WSL2 ==="

# 1. Update system and install required system tools
echo "--> Updating packages and installing prerequisites..."
sudo apt-get update -y
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# 2. Set up Docker's official GPG key securely
echo "--> Adding Docker GPG key..."
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 3. Add the stable Docker repository to apt sources
echo "--> Setting up the stable repository..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 4. Install Docker packages
echo "--> Installing Docker Engine, Containerd, and Compose..."
sudo apt-get update -y
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 5. Grant Docker permissions to the current user
echo "--> Adding user '$USER' to the docker group..."
sudo usermod -aG docker "$USER"

# 6. Start the Docker service based on available init system
echo "--> Initializing Docker service..."
if pidof systemd >/dev/null; then
    echo "    Systemd detected. Enabling and starting Docker daemon..."
    sudo systemctl enable docker
    sudo systemctl start docker
else
    echo "    Classic init detected. Starting Docker service manually..."
    sudo service docker start
fi

echo "========================================================="
echo " Installation complete!"
echo " IMPORTANT: Run 'newgrp docker' or restart your terminal"
echo " to apply permissions before running docker without sudo."
echo "========================================================="