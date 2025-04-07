# mujoco_ros2_control

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository can be tought as the equivalent of [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) but for the [Mujoco](https://mujoco.readthedocs.io/en/stable/overview.html) simulator.
The implementation is started from [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) but differs from it since it wrap the original code around a [Mujoco plugin](https://mujoco.readthedocs.io/en/stable/programming/extension.html#engine-plugins). 
The Mujoco Plugin instantate a [ros2_control](https://control.ros.org/rolling/index.html) controller manager that loads the [hadrware interface](mujoco_ros2_control/src/mujoco_system.cpp) as a plugin. 

Finally, it can be combined with [mujoco_ros_utils](https://github.com/pucciland95/mujoco_ros_utils) to increase its capabilities and get in Ros2 ecosystem other data like camera streams or object poses.

## Installation Guide

Follow these steps to install and run the project locally.

### Prerequisites

Make sure you have the following software installed if you are running on the local machine:

- [ROS](https://docs.ros.org/)
- [Mujoco](https://mujoco.org/)

### Package Install

Before build this package configure environment variable for mujoco directory.

```bash
export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x
```

You can now compile the package using the following commands.

```bash
cd mujoco_ros2_control
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

## Usage

TODO