# v-hsc-ros-bridge

## Introduction
This repository provides a ROS package named `ros_bridge` that can be used to communicate with the Webots server while using the Player/Client API.

It requests and receives sensor data (specified in the `src/ros_bridge/resources/devices.json`) over specific topics and subscribes to actuator command messages(only position control available at the moment). Node `webots_controller` is responsible for these operations.

This node is currently available only for ROS 2.

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

Run the node using this command:

`ros2 run ros_bridge webots_controller --ros-args -p host:="127.0.0.1" -p port:=10001`

(you can replace host and port with your preferred ones)

Done, now you can access sensor data and publish commands to be performed on the robot.