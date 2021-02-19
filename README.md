<!--
# Table of Contents 
- [Moving-Kinova](#Moving-Kinova) 
  - [Robot-Raconteur](#Robot-Raconteur)  
    - [Setup](#Setup) 
    - [Initialize](#Initialize)
  - [Aruco-ros](#Aruco-ros)
    - [Setup](#Setup)
  - [Fiducials](#Fiducials)
    - [Setup](#Setup)
  - [Arm and Wrist Camera](#Arm-and-Wrist-Camera)
- [Kinect-camera](#Kinect-camera)
  - [Run-camera](#Run-camera)
  - [Detect-tag](#detect-tag)
  - [Show images](#Show-images)
- [Assets](#Assets)
  - [Training-Data](#Training-Data)
- [Dependency](#Dependency)
  - [PointCloud Library](#PointCloud-Library)
  - [Moveit](#Moveit)  -->
# OARbot
ROS packages for OARbot which includes Kinova robotic arm and omni-directional moving base

## Installation

The current code is tested on ROS Kinetic. First install the tools and general ros packages such as python-catkin-tools, python-wstool, ros_control and realsense SDK. Second clone this repository and build locally

<!-- then use the
`OARbot-dependency.rosinstall` file to pull down the required dependencies. -->

```bash
# install tools and general packages
sudo apt install python-wstool
sudo apt-get install ros-kinetic-ddynamic-reconfigure
sudo apt install ros-kinetic-graph-msgs
sudo apt-get install ros-kinetic-catkin python-catkin-tools 

# Make a ROS WS and cd into the src/ folder
mkdir -p ~/oarbot_ws/src && cd ~/oarbot_ws/src
git clone https://github.com/Hubert51/OARbot.git

# Build the workspace (do it in Release or RelWithDebInfo)
cd ~/oarbot_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Manipulation in Gazebo Simulation(need to be completed)
<!--
### Base
#### First method(terminal input)
Initialize the gazebo world and control the base with terminal
```
# terminal 1:
roslaunch oarbot_description oarbot.launch
# terminal 2:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
-->

### Demo1: show the OARbot in gazebo and move the base to hit the cola can.

```bash
# source the config files
source /opt/ros/kinetic/setup.bash
source ~/oarbot_ws/devel/setup.bash

## open two terminals
# terminal 1:
roslaunch oarbot_description oarbotGoToPoint.launch
# terminal 2:
rosservice call /go_to_point_switch "data: true" 
```
<<<<<<< HEAD

[![Demo move OARbot in gezebo](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://youtu.be/OVhhgOve-hg)

=======
[![Demo move OARbot in gezebo](https://youtu.be/OVhhgOve-hg)]
>>>>>>> 63775272879c351991725ed6611616a58b3c2250

<!--

## Aruco-ros
* Software package and ROS wrappers of the Aruco Augmented Reality marker detector library. Refer [this](http://wiki.ros.org/aruco) for more details
### Setup
1. To install the required packages from binary packages
```
sudo apt-get install ros-kinetic-usb-cam ros-kinetic-aruco-ros
```
2. Aruco size for the kinect is 17.7cm. Firstly, we generate the .svg file and change the size in the property.

## Fiducials
* This package provides a system that allows a robot to determine its position and orientation by looking at a number of fiducial markers (similar to QR codes) that are fixed in the environment of the robot. Refer [this](http://wiki.ros.org/fiducials) for more details
### Setup
1. To install the fiducial software from binary packages
```
sudo apt-get install ros-kinetic-fiducials
```
## Arm and Wrist Camera
The kinova arm and the camera on the wrist
### Initialize
To initialize the camera SDK, kinova SDK and tag detection.  
`roslaunch assistiverobot bringup_arm_camera_tag.launch`


## Moveit
* Official supported ros version is indego in ubuntu14.04. This is [documentation](https://github.com/Kinovarobotics/kinova-ros/wiki/MoveIt). If you want to use moveit in kinetic in ubuntu16.04. Some changes need to be applied. The scripts required to use moveit are 
```
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=*robot_type*  
roslaunch robot_name_moveit_config robot_name_demo.launch  
rosrun kinova_driver pid_traj_action_server.py
```
And we also need to modify the code in `pid_traj_action_server.py` if the arm is not `j2s7s300`. Refer [this solution](https://github.com/Kinovarobotics/kinova-ros/issues/257) to modify 

### Box orientation
* The matlab and python have same quaternion format \[w, x, y, z\]


# Kinect-camera
Two kinect cameras fix on the top of room to provide the image and depth data for the system  

## Run-camera
In our system, we have two cameras
```
roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl sensor:=008097451747 base_name:=kin1
roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl sensor:=501004442442 base_name:=kin2
```

## Detect-tag(same as above)
```
roslaunch aruco_detect aruco_detect.launch
```

## Show-images
`rosrun image_view image_view image:=/fiducial_images`

Optional:
```
rosrun kinect2_viewer kinect2_viewer kin1
rosrun kinect2_viewer kinect2_viewer kin2
```

# Change Log:
* 2020-08-03: Remove the `moveit`, `moveit_msgs`, `moveit_tutorials`, `moveit_visual_tools`  package from the project. 

# Assets
## Training-Data
* [Objects in the fridge](https://drive.google.com/drive/folders/1nERUeKihDFWaOkOLvG9-ORnOm8ObeKN1?usp=sharing)

# Dependency(Reference)
1. PointCloud Library
  * [python-pcl github repo](https://github.com/strawlab/python-pcl)
  * [python-pcl website](http://strawlab.github.io/python-pcl/)
2. Moveit
  * [planning_scene_interface.py](http://docs.ros.org/jade/api/moveit_commander/html/planning__scene__interface_8py_source.html)
  * [moveit_commander.move_group.MoveGroupCommander Class Reference](http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a0e95859080ce005ee4d907b8dac7d8e3)
3. Omni-directional Base
  * [Mecanum Wheels(video tutorial)](https://www.youtube.com/watch?v=sb7FoOGzb8E&t=319s)

  -->
