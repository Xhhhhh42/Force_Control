# Impedance Controller
Maintainer: Yuchen Xia

Main Supervisor: Jiayun Li

## Introduction 
With this Repo, you can control Franka Emika robot( Version: Panda, see below ) in **Joint Space** or in **Task Space** with Null Space Damping.

Refering build section fo setup the package:
- [Prerequisites](#prerequisites)
- [Build](#build)

Following the brief introduction for different impedance controller in the Repo:
- [Joint Space](#joint-space-impedance-controller)
- [Task Space](#task-space-impedance-controller-with-null-space-damping)

<img src="./doc/roboter_arm.jpg" alt="Panda Robot" width="500" height="500">


## Prerequisites
1. __Ubuntu and ROS__

This package is intended to be used with Ubuntu 22.04 and ROS Humble.

2. __Dependencies__

First you need to install the following dependencies:
<!-- * [libfranka](https://github.com/frankaemika/libfranka) -->
* [franka_ros2](https://github.com/frankaemika/franka_ros2)
( The franka_ros2 repo contains a ROS 2 integration of libfranka. See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs/franka_ros2.html) for more information of franka_ros2. )

3. __Communication with Real Robot Hardware__

3.1 Open the browser with `https://192.168.2.55/desk/`.
3.2 Open fail safe locking system.
3.3 Activate FCI.
3.4 Operations in `Execution` Mode.
3.5 Indicator light shows **green**.


## Build
Create a ROS 2 workspace:

```
mkdir -p ~/franka_ros2_ws/src
```


If already done so, clone repo `Force_Control` to your workspace and build packages:

```
cd franka_ros2_ws/src
git clone https://...
cd ..

colcon build force_control
or colcon build --packages-select force_control // Compile specific package

source install/setup.bash
```

## Run
Run the launch file
```
ros2 launch force_control impedance_controller.launch.py robot_ip:=192.168.2.55
```

## Parameter




