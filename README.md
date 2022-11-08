# HLVS Player

[![Build and Test (rolling)](https://github.com/ros-sports/hlvs_player/actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=main)](https://github.com/ros-sports/hlvs_player/actions/workflows/build_and_test_rolling.yaml?query=branch:main)

## Introduction
This repository provides a ROS package named `hlvs_player` that can be used to communicate with the Webots server while using the Player/Client API.

It requests and receives sensor data (specified in the `src/hlvs_player/resources/devices.json`) over specific topics and subscribes to actuator command messages(only position control available at the moment). Node `hlvs_player` is responsible for these operations.

This node is currently available only for ROS 2.

## Configuration
The package comes with the default configuration for a Darwin-OP robot, but you can easily change it to your robot.
The `ressources/devices.json` contains the definition of the devices (sensors and actuators).
You can set the used network configuration and the ROS topic names via ROS parameters (see `launch/hlvs_player.launch`).
Remember to rebuild the package after changing the json file.

## Usage

Clone the repository:

`git clone https://github.com/robocup-hl-tc/v-hsc-ros-bridge.git`

To access to the ROS2 commands in a bash, source ROS with this command:

`source /opt/ros/$ROS_DISTRO/setup.bash`

(replace $ROS_DISTRO with your ROS distribution, e.g. foxy)

Move to the repository:

`cd v-hsc-ros-bridge`

You can install the dependencies using:

`rosdep install -i --from-path src -y`

build the package with this command:

`colcon build`

In the root of the repository, source your overlay using:

`. install/local_setup.bash`

Run the node using one of these commands:

`ros2 run hlvs_player hlvs_player --ros-args -p host:="127.0.0.1" -p port:=10001`
`ros2 launch hlvs_player hlvs_player.launch`

(you can replace host and port with your preferred ones)

Done, now you can access sensor data and publish commands to be performed on the robot.
