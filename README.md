# Yesense ROS2 Driver Package

This repository contains the ROS2 driver for the Yesense IMU device. It provides a serial communication node that reads IMU data from the device and publishes it as ROS2 messages.

## Installation

Before building the package, make sure to install the required dependencies:

```bash
sudo apt-get update
sudo apt-get install ros-humble-serial-driver ros-humble-io-context ros-humble-asio-cmake-module
```

## Usage
1. Build the package with your ROS2 workspace:  
   ```bash
    colcon build --packages-select yesense_ros2_driver
   ```
2. Source the workspace:
   ```bash
    source install/setup.bash
   ```
3. Connect the Yesense IMU device and ensure the udev rule is installed:
   ```bash
    sudo cp udev/imu.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
   ```
4. Launch the driver node:
   ```bash
    ros2 launch yesense_ros2_driver yesense_ros2_driver.launch.py
   ```