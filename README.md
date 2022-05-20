[![ci](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/ci.yml/badge.svg)](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/ci.yml)
[![lint](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/lint.yml/badge.svg)](https://github.com/HarvestX/MGS1600GY-ROS2/actions/workflows/lint.yml)

# MGS1600GY-ROS2
RoboteQ linetrace module controller ROS2 driver


## Requirements
- Linux OS
  - [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- ROS 2
  - [Galactic Geochelone](https://index.ros.org/doc/ros2/Installation/Galactic/)


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
