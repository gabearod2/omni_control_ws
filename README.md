# SHPE Omnidirectional Control Workspace

## Introduction and Scope

This repository contains a ROS2 Humble workspace allowing for real time deployment of SSD mobilenet for object capture. Students and other contirbutors can create their own controllers to test and deploy on the robot. 

## Startup

First clone this repo into your workspaces directory. Then go to [Jetson Containers](https://github.com/dusty-nv/jetson-inference/tree/master) and setup your Jetson to create the container using the command below:

```bash
$ git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ docker/run.sh
```

Then, run the container and build:

```bash
docker run -it --rm -v /home/shpe/workspaces:/workspaces omni_control_container bash -c "cd /workspaces && ls"
cd omni_control_ws
colcon build
```

Then, run the launch file:

```bash
ros2 launch omni_launch omni_control.launch.py
```

Check out Embry-Riddle's SHPE chapter [here](https://eraushpe.org/).
