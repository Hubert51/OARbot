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

### Initialize 
1. For the control of arm  
`rosrun kinova_rr_bridge jointController_host.py --port 4567`

2. For open the camera  
`rosrun kinova_rr_bridge camera_host.py --port 2345`

3. For initialize the code to accept joystick as user input  
`rosrun kinova_rr_bridge ui_host.py --port 7890`

4. For frame transformation  
`rosrun kinova_rr_bridge peripherals_host.py --port 1234`

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
