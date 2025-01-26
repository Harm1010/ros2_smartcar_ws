
# Smart Car ROS2 Package
=====================================

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://index.ros.org/doc/humble/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

This package provides a simulation of a smart car using ROS2 Humble. It includes nodes for publishing joint states, transforming coordinates, and simulating the car's movement.

## Dependencies

* ROS2 Humble
* Python 3.8 or later
* `gazebo_ros` package for simulation
* `robot_state_publisher` package for publishing robot state

## Nodes

### joint_state_pub

Publishes joint states for the car's wheels.

* Subscribes to: `steering_angle_rad`, `engine_speed_rpm`
* Publishes: `joint_states`

### transform_pub

Publishes transformation messages for the car's front wheels.

* Subscribes to: `steering_angle_rad`
* Publishes: `transform_stamped`

## Launch Files

### nav2.launch.py

Launches the navigation stack with the smart car simulation.

### localization.launch.py

Launches the localization node with the smart car simulation.

## Configuration Files

### config/ekf.yaml

Configuration file for the Extended Kalman Filter (EKF) used in localization.

### config/nav2_params.yaml

Configuration file for the navigation stack.

### config/smalltown_world.yaml

Configuration file for the simulation world.

## URDF Files

### urdf/smartcar.urdf.xacro

URDF file for the smart car model.

## World Files

### world/smalltown.world

World file for the simulation environment.

## Installation

To install this package, clone the repository and build it using the following commands:
```bash
git clone https://github.com/Harm1010/smart_car.git
cd smart_car
colcon build
```
## Usage

To run the simulation, use the following command:
```bash
ros2 launch smart_car nav2.launch.py
```
This will launch the navigation stack with the smart car simulation. You can then use RViz to visualize the simulation and control the car using the `steering_angle_rad` and `engine_speed_rpm` topics.

## Contributing

Contributions to this package are welcome. Please submit pull requests or issues to the repository.

## License

This package is licensed under the Apache-2.0 license.

## Author

Harm1010
