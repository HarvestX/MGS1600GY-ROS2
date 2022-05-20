[![ci](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/ci.yml/badge.svg)](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/ci.yml)
[![lint](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/lint.yml/badge.svg)](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/lint.yml)

# MGS1600GY-ROS2
RoboteQ line sensor module controller for ROS2.

![Image](https://github.com/HarvestX/MGS1600GY-ROS2/blob/main/media/MGS1600GY.png?raw=true)



## Requirements
- Linux OS
  - [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- ROS 2
  - [Galactic Geochelone](https://index.ros.org/doc/ros2/Installation/Galactic/)

## Setup
Add user to dialout group.
```bash
sudo adduser $USER dialout
```
User will need to log out & log back in again for this to take effect.

## Install
### Locate package in workspace
```bash
mkdir -p ~/<Your Workspace>/src
cd ~/<Your Workspace>/src
git clone git@github.com:HarvestX/MGS1600GY-ROS2.git
```

### Install dependencies
```bash
source /opt/ros/galactic/setup.bash
cd ~/<Your Workspace>
rosdep update
rosdep install -r -y -i --from-paths ./src/MGS1600GY-ROS2 --rosdistro $ROS_DISTRO
```

### Build source
```bash
cd ~/<Your Workspace>
colcon build
```

## Launch
```bash
source ~/<Your Workspace>/install/setup.bash
ros2 launch mgs1600gy_bringup mgs1600gy.launch.py dev:=<Your Path to Device>
```
