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
   
