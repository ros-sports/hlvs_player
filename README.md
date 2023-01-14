# Humanoid League Virtual Season Player

[![Build and Test (rolling)](https://github.com/ros-sports/hlvs_player/actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=main)](https://github.com/ros-sports/hlvs_player/actions/workflows/build_and_test_rolling.yaml?query=branch:main)

## Introduction
This repository provides a ROS 2 package named `hlvs_player` that can be used to communicate with the Webots server while using the Player/Client API for the Humanoid League Virtual Season.

It requests and receives sensor data (specified in the `src/hlvs_player/resources/devices.json`) over specific topics and subscribes to actuator command messages(only position control available at the moment). Node `hlvs_player` is responsible for these operations.

This node is currently available only for ROS 2.

## Configuration
The package comes with the default configuration for a Darwin-OP robot, but you can easily change it to your robot.
The `resources/devices.json` contains the definition of the devices (sensors and actuators).
You can set the used network configuration and the ROS topic names via ROS parameters (see `launch/example.launch`).
Remember to rebuild the package after changing the json file.

## Usage

Source your ROS 2 installation:

```sh
source /opt/ros/rolling/setup.bash  # Replace "rolling" with your distro
```

In your ROS 2 workspace, clone the repository:

```sh
git clone https://github.com/ros-sports/hlvs_player.git src/hlvs_player
```

Install dependencies:

```sh
rosdep install --from-paths src --ignore-src --default-yes
```

Build the package:

```sh
colcon build
```

Open a new terminal and source your overlay using:

```sh
source install/local_setup.bash
```

Set the simulator address:

```sh
# Replace host and port with appropriate values
export ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10001
```

Run the node using one of these commands:

```sh
ros2 run hlvs_player hlvs_player
ros2 launch hlvs_player example.launch
```

Now you can access sensor data and publish commands to be performed on the robot.
