# ROS2 Jazzy Installation Guide for Ubuntu 24.04

## System Requirements

### Recommended Setup
- **OS:** Ubuntu 24.04.3 LTS (Noble)
- **Architecture:** x86_64 (64-bit)
- **ROS2 Version:** Jazzy (Latest - May 2024)
- **RAM:** Minimum 2GB (4GB+ recommended for development)
- **Disk Space:** Minimum 5GB free space
- **Network:** Required for package downloads and updates

### Your Current System
```
Distributor ID: Ubuntu
Description: Ubuntu 24.04.3 LTS
Release: 24.04
Codename: noble
```

## Prerequisites

Before starting the installation, ensure you have:
- Ubuntu 24.04 LTS (Noble) with full system updates
- Administrator/sudo access
- Active internet connection
- Basic command-line familiarity

---

## Installation Steps

### Step 1: Update System Packages

```bash
# Update package index
sudo apt update

# Upgrade existing packages
sudo apt upgrade -y

# Install required dependencies
sudo apt install -y \   
  software-properties-common \
  curl \
  lsb-release \
  gnupg
```

### Step 2: Add ROS2 Repository and GPG Key

```bash
# Add the universe repository (if not already added)
sudo add-apt-repository universe -y

# Download and add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository to APT sources
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists with new repository
sudo apt update
```

### Step 3: Install ROS2 Jazzy Desktop Full

```bash
# Install ROS2 Jazzy with all development tools
sudo apt install -y ros-jazzy-desktop-full

# Optional: Install additional ROS2 packages
sudo apt install -y \
  ros-jazzy-dev-tools \
  ros-jazzy-demo-nodes-cpp \
  ros-jazzy-demo-nodes-py
```

### Step 4: Install Development Tools

```bash
# Install essential development tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  vim \
  tmux
```

### Step 5: Initialize and Update rosdep

```bash
# Initialize rosdep database (first time only)
sudo rosdep init

# Update rosdep database
rosdep update

# Verify rosdep installation
rosdep list | head -20
```

### Step 6: Configure Shell Environment

```bash
# Add ROS2 setup to bashrc for automatic sourcing
echo "# ROS2 Jazzy Setup" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Apply changes to current session
source ~/.bashrc

# Verify ROS2 is properly sourced
echo "ROS2 Setup Complete!"
printenv | grep -i ros
```

### Step 7: Verify Installation

```bash
# Check ROS2 version
ros2 --version

# Check ROS2 environment variables
printenv | grep ROS

# List available ROS2 packages
ros2 pkg list | head -20

# Test ROS2 with demo nodes

# Terminal 1 - Run talker node:
ros2 run demo_nodes_cpp talker

# Terminal 2 - Run listener node (open new terminal):
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

### Step 8: Create ROS2 Workspace (Recommended)

```bash
# Create workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone or create packages in src/

# Build the workspace
colcon build

# Source the workspace overlay
source ~/ros2_ws/install/setup.bash

# Optional: Add to bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Post-Installation Configuration

### Enable Tab Completion (Optional)

```bash
# Install argcomplete for command completion
sudo apt install python3-argcomplete

# Add completion to bashrc
echo "eval \"\$(register-python-argcomplete3 ros2)\"" >> ~/.bashrc
source ~/.bashrc
```

### Install Additional Tools (Optional)

```bash
# Install ROS2 CLI tools and extensions
sudo apt install -y \
  ros-jazzy-rqt* \
  ros-jazzy-rviz2 \
  ros-jazzy-gazebo-ros \
  ros-jazzy-tf2-tools

# Install Python development tools
pip install --upgrade pip setuptools wheel

# Install commonly used Python packages for ROS2
pip install --user numpy scipy matplotlib
```

---

## Troubleshooting Guide

### Issue: `ros2 command not found`

**Cause:** ROS2 environment not sourced

**Solution:**
```bash
# Manually source the setup script
source /opt/ros/jazzy/setup.bash

# Verify it's in bashrc
grep "source /opt/ros/jazzy/setup.bash" ~/.bashrc
```

### Issue: `rosdep` initialization fails

**Cause:** Network connectivity or permission issues

**Solution:**
```bash
# Check network connectivity
ping google.com

# Retry rosdep initialization
sudo rosdep init
rosdep update

# Check rosdep status
rosdep db status
```

### Issue: Permission denied during installation

**Cause:** Insufficient permissions

**Solution:**
```bash
# Ensure sudo privileges
groups $USER

# If not in sudo group, add user (run as existing sudo user):
sudo usermod -aG sudo username

# Log out and log back in for changes to take effect
```

### Issue: Unmet dependencies

**Cause:** Package cache issues or incomplete installation

**Solution:**
```bash
# Update and upgrade all packages
sudo apt update
sudo apt upgrade -y

# Fix broken dependencies
sudo apt install -f

# Clean package cache
sudo apt clean
sudo apt autoclean

# Retry ROS2 installation
sudo apt install -y ros-jazzy-desktop-full
```

### Issue: Package not found for ROS2 Jazzy

**Cause:** Repository not properly added or cache not updated

**Solution:**
```bash
# Verify ROS2 repository is configured
cat /etc/apt/sources.list.d/ros2.list

# Verify GPG key is installed
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# Update package cache
sudo apt update

# Clear and refresh cache
sudo apt clean
sudo apt update
```

### Issue: Multiple ROS2 distributions conflict

**Cause:** Multiple ROS2 setups in bashrc or environment

**Solution:**
```bash
# Check bashrc for multiple ROS2 sourcing
grep "source /opt/ros" ~/.bashrc

# Remove duplicate lines from bashrc
# Edit ~/.bashrc and keep only one ROS2 source line

# Restart terminal to apply changes
```

---

## Common Commands Reference

### ROS2 Package Commands

```bash
# List all available packages
ros2 pkg list

# Find a specific package
ros2 pkg list | grep package_name

# Get information about a package
ros2 pkg prefix package_name
```

### ROS2 Node Commands

```bash
# List running nodes
ros2 node list

# Get information about a node
ros2 node info node_name

# Run a node
ros2 run package_name executable_name
```

### ROS2 Topic Commands

```bash
# List all topics
ros2 topic list

# Show topic data types
ros2 topic list -t

# Echo (display) topic messages
ros2 topic echo topic_name

# Get topic information
ros2 topic info topic_name
```

### Colcon Build Commands

```bash
# Build all packages in workspace
colcon build

# Build specific package
colcon build --packages-select package_name

# Build with parallel jobs
colcon build --parallel-workers 4

# Clean build artifacts
colcon clean packages

# Build and test
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## ROS2 Distribution Reference

| Distribution | Ubuntu LTS | Release Date | EOL Date    | Status          |
|-------------|-----------|--------------|-------------|-----------------|
| **Jazzy**   | 24.04     | May 2024     | May 2029    | **Latest**      |
| Iron        | 24.04     | May 2023     | Nov 2024    | Maintenance     |
| Humble      | 22.04     | May 2022     | May 2027    | LTS Support     |
| Foxy        | 20.04     | Jun 2020     | May 2025    | Maintenance     |
| Galactic    | 20.04     | May 2021     | Dec 2022    | EOL             |

---

## Additional Resources

### Official Documentation
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
- [ROS2 Jazzy Complete Documentation](https://docs.ros.org/en/jazzy/)
- [ROS2 Beginner CLI Tools Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- [ROS2 Concepts Documentation](https://docs.ros.org/en/jazzy/Concepts.html)

### Learning Resources
- [ROS Index - Package Search](https://index.ros.org/)
- [ROS Discourse - Community Forum](https://discourse.ros.org/)
- [ROS2 GitHub Repository](https://github.com/ros2/ros2)
- [ROS2 Design Articles](https://design.ros2.org/)

### Tools and Extensions
- [RViz2 - 3D Visualization](https://github.com/ros-visualization/rviz)
- [Gazebo - Simulator](https://gazebosim.org/)
- [rqt - GUI Tool Framework](https://wiki.ros.org/rqt)

---

## Quick Verification Checklist

After installation, verify everything is working:

- [ ] ROS2 command is available: `ros2 --version`
- [ ] Environment variables set: `echo $ROS_DISTRO` (should output `jazzy`)
- [ ] Packages found: `ros2 pkg list | wc -l` (should show package count)
- [ ] Demo nodes work: Successfully ran talker/listener demo
- [ ] Workspace created: `~/ros2_ws` directory exists (if needed)
- [ ] Development tools installed: `colcon --version` works
- [ ] rosdep initialized: `rosdep db status` shows valid database

---

## Next Steps

1. **Learn ROS2 Basics:** Follow the [ROS2 beginner tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
2. **Create Your First Package:** Practice by creating a simple C++ or Python package
3. **Explore Example Packages:** Clone and build existing ROS2 repositories
4. **Join the Community:** Ask questions on [ROS Discourse](https://discourse.ros.org/)
5. **Set Up Your Development Environment:** Configure IDE and development tools

---

## Support

If you encounter issues:
1. Check the [Troubleshooting Guide](#troubleshooting-guide) above
2. Search [ROS Discourse](https://discourse.ros.org/)
3. Check [ROS2 GitHub Issues](https://github.com/ros2/ros2/issues)
4. Review [Official ROS2 Documentation](https://docs.ros.org/)

**Last Updated:** November 2025
**ROS2 Version:** Jazzy
**Ubuntu Version:** 24.04 LTS (Noble)
