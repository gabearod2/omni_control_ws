# SHPE Omnidirectional Control Workspace

## Introduction and Scope

This repository contains a ROS2 Humble workspace allowing for real time deployment of SSD mobilenet for object capture. Students and other contirbutors can create their own controllers to test and deploy on the robot. 

## Startup

First clone this repo into your workspaces directory. Then go to [Jetson Containers](https://github.com/dusty-nv/jetson-inference/tree/master) and setup your Jetson to create the container using the command below:

```bash
$ git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ docker/run.sh --ros=humble
```

I will have to change the dependencies of this ROS package to include that of the jetson-inference ros.

TODO: Add ability to not have an output. 

```bash
ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0
```

Then, run the launch file:

```bash
ros2 launch omni_launch omni_control.launch.py
```

Check out Embry-Riddle's SHPE chapter [here](https://eraushpe.org/).
