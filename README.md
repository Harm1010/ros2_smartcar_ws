
# ROS2 Humble Workspace
==========================

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://index.ros.org/doc/humble/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

This is a ROS2 Humble workspace containing packages for a smart car simulation.

## Packages

* `smart_car`: Simulation of a smart car using ROS2.
* `smartcar_msgs`: Custom message definitions for the smart car simulation.

## Dependencies

* ROS2 Humble
* Python 3.8 or later

## Installation

To install this workspace, clone the repository and build it using the following commands:
```bash
git clone https://github.com/harm1010/ros2_humble_workspace.git
cd ros2_humble_workspace
colcon build
```

## Usage

To use this workspace, source the `install/setup.bash` file and run the desired package:
```bash
source install/setup.bash
ros2 launch smart_car nav2.launch.py
```

## Contributing

Contributions to this workspace are welcome. Please submit pull requests or issues to the repository.

## License

This workspace is licensed under the Apache-2.0 license.

## Author

Harm1010
