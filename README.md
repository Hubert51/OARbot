# Table of Contents 
- [Moving-Kinova](#Moving-Kinova) 
  - [Robot-Raconteur](#Robot-Raconteur)  
    - [Setup](#Setup) 
  - [Aruco-ros](#Aruco-ros)
    - [Setup](#Setup)
  - [Fiducials](#Fiducials)
    - [Setup](#Setup)

# Moving-Kinova
ROS packages for Kinova robotic arms and its moving base

## Robot-Raconteur
* a middleware for users to control kinova arm
### Setup
1. Initialize robot raconteur package in source
```
cd ~/ros_ws/src
mkdir kinova_rr_bridge && cd kniova_rr_bridge 
```

2. Refer the sample in this project to create [XML file]() and [camke file]() in kinova_rr_bridge.

## Aruco-ros
* Software package and ROS wrappers of the Aruco Augmented Reality marker detector library. Refer [this](http://wiki.ros.org/aruco) for more details
### Setup
1. To install the required packages from binary packages
```
sudo apt-get install ros-kinetic-usb-cam ros-kinetic-aruco-ros
```

## Fiducials
* This package provides a system that allows a robot to determine its position and orientation by looking at a number of fiducial markers (similar to QR codes) that are fixed in the environment of the robot. Refer [this](http://wiki.ros.org/fiducials) for more details
### Setup
1. To install the fiducial software from binary packages
```
sudo apt-get install ros-kinetic-fiducials
```

## Moveit
* Official supported ros version is indego in ubuntu14.04. This is [documentation](https://github.com/Kinovarobotics/kinova-ros/wiki/MoveIt). If you want to use moveit in kinetic in ubuntu16.04. Some changes need to be applied. The scripts required to use moveit are 
```
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=*robot_type*  
roslaunch robot_name_moveit_config robot_name_demo.launch  
rosrun kinova_driver pid_traj_action_server.py
```
And we also need to modify the code in `pid_traj_action_server.py` if the arm is not `j2s7s300`. Refer [this solution](https://github.com/Kinovarobotics/kinova-ros/issues/257) to modify 
