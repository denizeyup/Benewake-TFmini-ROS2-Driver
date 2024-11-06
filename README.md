# Benewake-TFmini-ROS2-Driver
This repository provides a ROS2-compatible driver for the Benewake TFmini LiDAR sensor. It is a port of the original ROS driver available at [TFmini-ROS on GitHub](https://github.com/TFmini/TFmini-ROS), adapted for the ROS2 environment.

## Overview

The TFmini ROS2 driver reads data from the TFmini LiDAR sensor and publishes the distance information as ROS2 messages in the `sensor_msgs/Range` format.

## Setup and Installation

To set up the TFmini ROS2 driver, follow these steps:

1. **Clone the repository into your ROS2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/denizeyup/Benewake-TFmini-ROS2-Driver.git

2. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build

3. **Source the workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash

## Core Features

**Launching the Driver**  
To start the TFmini ROS2 node, use the following command:
```bash
ros2 run tfmini_ros2 tfmini_ros_node
```

**Topics**  
- **Published Topic:** `/tfmini/range`  
- **Message Type:** `sensor_msgs/Range`  
- **Description:** Publishes the measured distance data from the TFmini LiDAR sensor.

## Code Details

The code in this repository includes two main source files:  
- **tfmini_ros2_node.cpp**: Initializes the ROS2 node, handling data reading and publishing for the TFmini LiDAR sensor.  
- **TFmini.cpp**: Contains the core driver code for interfacing with the TFmini hardware, reading sensor data, and outputting it in a format compatible with ROS2.

### Package Configuration

The following files configure the package for ROS2:  
- **CMakeLists.txt**: Defines the build configuration, dependencies, and installation paths.  
- **package.xml**: Lists package metadata, dependencies (e.g., `rclcpp`, `sensor_msgs`), and specifies build details.

## Dependencies

This ROS2 driver depends on the following packages:  
- **rclcpp**  
- **sensor_msgs**  
Ensure these dependencies are installed in your ROS2 environment.

## Additional Information

This code is an adaptation of the **TFmini-ROS** repository, making it compatible with ROS2. Please refer to the original repository for more information on the ROS1 version.

## License

This project is licensed under the **Apache License 2.0.**

