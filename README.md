# SHPE Omnidirectional Control Workspace

## Introduction and Scope

This repository contains a ROS2 Humble workspace allowing for real time deployment of different pre-trained models for object capture. Students and other contributors can create their own controllers to test and deploy on the robot. Development is underway for human following and more.

## Startup

First go to [Jetson Containers](https://github.com/dusty-nv/jetson-inference/tree/master) and setup your Jetson to create the container using the command below:

```bash
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference &&
cd jetson-inference &&
docker/run.sh --ros=humble
```
Then, clone this repo into your workspaces directory:

```bash
git clone https://github.com/gabearod2/omni_control_ws
```

Then, run the launch file for the object detection for jetson-inference.

```bash
ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0

```

Then, run the launch file for the controller based on the bounding box messages.

```bash
ros2 launch omni_launch omni_control.launch.py
```

Check out Embry-Riddle's SHPE chapter [here](https://eraushpe.org/).
