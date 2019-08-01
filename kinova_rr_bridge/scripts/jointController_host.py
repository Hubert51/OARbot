#!/usr/bin/env python2
import rospy
import numpy as np
import cv2

import math
from math import pi
import tf
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_from_matrix, inverse_matrix, euler_from_quaternion, euler_from_matrix

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import thread
import threading

# camera
from camera_host import KinovaCamera

data = None
prefix = "j2n6s300_"
# right joint position
position = [4.678701590491841, 3.8050904770432625, 0.9413227831121705, -2.2098248929946633, 1.6683761470320537, 8.621530226481784, 0.0012319970456220702, 0.0024639940912441404, 0.0]



import RobotRaconteur as RR

import sys, argparse
import thread
import threading

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from p2os_msgs.msg import MotorState
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

from scipy import stats



""" Global variable """
arm_joint_number = 0
finger_number = 0
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentFingerPosition = [0.0, 0.0, 0.0]

def gripper_client(finger_positions):
    print 'in function'
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'
    action_address = '/j2n6s300_driver/fingers_action/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None


def getCurrentFingerPosition(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/finger_position'
    rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, setCurrentFingerPosition)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
    print 'obtained current finger position '


def setCurrentFingerPosition(feedback):
    global currentFingerPosition
    currentFingerPosition[0] = feedback.finger1
    currentFingerPosition[1] = feedback.finger2
    currentFingerPosition[2] = feedback.finger3


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive fingers to command position')
    parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
    parser.add_argument('unit', metavar='unit', type=str, default='turn',
                        choices={'turn', 'mm', 'percent'},
                        help='Unit of finger motion command, in turn[0, 6800], mm[0, 9.45], percent[0,100]')
    parser.add_argument('finger_value', nargs='*', type=float, help='finger values, length equals to number of fingers.')

    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display finger values in alternative convention(turn, mm or percent)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    return args_


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit_, finger_value_, relative_):
    """ Argument unit """
    global currentFingerPosition

    # transform between units
    if unit_ == 'turn':
        # get absolute value
        if relative_:
            finger_turn_absolute_ = [finger_value_[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_value_

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]

    elif unit_ == 'mm':
        # get absolute value
        finger_turn_command = [x/1000 * finger_maxTurn / finger_maxDist for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    elif unit_ == 'percent':
        # get absolute value
        finger_turn_command = [x/100.0 * finger_maxTurn for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in
                                     range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn_, finger_meter_, finger_percent_


def verboseParser(verbose_, finger_turn_):
    """ Argument verbose """
    if verbose_:
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
        print('Finger values in turn are: ')
        print(', '.join('finger{:1.0f} {:4.0f}'.format(k[0] + 1, k[1]) for k in enumerate(finger_turn_)))
        print('Finger values in mm are: ')
        print(', '.join('finger{:1.0f} {:2.1f}'.format(k[0]+1, k[1]*1000) for k in enumerate(finger_meter_)))
        print('Finger values in percentage are: ')
        print(', '.join('finger{:1.1f} {:3.1f}%'.format(k[0]+1, k[1]) for k in enumerate(finger_percent_)))



kinova_servicedef="""
#Service to provide simple interface to Baxter
service KinovaJoint_interface

option version 0.4

struct BaxterImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end struct

struct CameraIntrinsics
    field double[] K
    field double[] D
end struct

struct ImageHeader
    field int32 width
    field int32 height
    field int32 step
end struct

struct ARtagInfo
    field double[] tmats
    field int32[] ids
end struct


object Kinova
    property uint8 camera_open

    # camera control functions
    function double[] getOri()
    function double[] getPos()
    function double cartesian_pose_client(double[] pos, double[] ori, double relative)
    function void closeFinger(double[] values)

    # function void openCamera()
    # function void closeCamera()
    # function void setExposure(int16 exposure)
    # function void setGain(int16 gain)
    # function void setWhiteBalance(int16 red, int16 green, int16 blue)
    # function void setFPS(double fps)
    # function void setCameraIntrinsics(CameraIntrinsics data)
    # function void setMarkerSize(double markerSize)
    
    # functions to acquire data on the image
    # function BaxterImage getCurrentImage()
    # function ImageHeader getImageHeader()
    # function CameraIntrinsics getCameraIntrinsics()
    # function double getMarkerSize()
    # function ARtagInfo ARtag_Detection()
    
    # pipe to stream images through
    # pipe BaxterImage ImageStream
    
end object

"""


class Kinova(object):
    """docstring for Kinova"""
    def __init__(self):
        self.address = "/j2n6s300_driver/out/tool_pose"
        rospy.init_node('kinova', anonymous = True)

        self._ee_pos = [0]*3
        self._ee_ori = [0]*4
        self._pose = None
        self._valid = False

        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        # image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, process) 

    def getPos(self):
        while self._valid == False:
            rospy.sleep(0.1)
            pass
        return self._ee_pos

    def getOri(self):
        while self._valid == False:
            rospy.sleep(0.1)
            pass
        return self._ee_ori

    def process(self, data):
        pose = data.pose
        self._ee_pos[0] = pose.position.x
        self._ee_pos[0] = pose.position.x
        self._ee_pos[1] = pose.position.y
        self._ee_pos[2] = pose.position.z
        self._ee_ori[0] = pose.orientation.x
        self._ee_ori[1] = pose.orientation.y
        self._ee_ori[2] = pose.orientation.z
        self._ee_ori[3] = pose.orientation.w

        self._valid = True

    def readEndEffectorPoses(self):
        l_pose = self._left.endpoint_pose()
        if l_pose:
            self._ee_pos[0] = l_pose['position'].x
            self._ee_pos[1] = l_pose['position'].y
            self._ee_pos[2] = l_pose['position'].z
            self._ee_ori[0] = l_pose['orientation'].w
            self._ee_ori[1] = l_pose['orientation'].x
            self._ee_ori[2] = l_pose['orientation'].y
            self._ee_ori[3] = l_pose['orientation'].z
        r_pose = self._right.endpoint_pose()


    ## @brief move the arm to desired pose in cartesianly
    ## @param position: the desired position, in meter. 
    def cartesian_pose_client(self, position, orientation, relative):
        """Send a cartesian goal to the action server."""
        action_address = '/' + prefix + 'driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        if int(relative) == 1:
            position = np.array(position) + np.array(self._ee_pos)

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])


        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            # print client.get_result()
            return 1
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return 0
            return None


    def closeFinger(self, values):

        print('Sending finger position ...')
        result = gripper_client(values)
        print('Finger position sent!')




    # worker function to request and update end effector data for baxter
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, self.process) 

        # while self._running:
        #     t1 = time.time()
        #     self.readEndEffectorPoses()
        #     self.readEndEffectorTwists()
        #     self.readEndEffectorWrenches()
        #     while (time.time() - t1 < 0.01):
        #         # idle
        #         time.sleep(0.001)


def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(description='Initialize Baxter moveit module.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on (will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the RobotRaconteur Node name
    RR.RobotRaconteurNode.s.NodeName="KinovaJointServer"

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
        RR.IPNodeDiscoveryFlags_LINK_LOCAL | RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(kinova_servicedef)
    
    #Initialize object
    kinova_obj = Kinova()
    
    RR.RobotRaconteurNode.s.RegisterService("Kinova", 
                "KinovaJoint_interface.Kinova", kinova_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/KinovaJointServer/Kinova" 
    raw_input("press enter to quit...\r\n")

    # baxter_obj.closeCamera()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
    # export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH