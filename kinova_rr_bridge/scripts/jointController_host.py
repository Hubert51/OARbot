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
from geometry_msgs.msg import Pose, PoseStamped, PoseArray


import thread
import threading
import time

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

# from scipy import stats

# moveit 
import moveit_commander

import tf_conversions

import tf2_ros
import geometry_msgs.msg
# import kinova_msgs.msg
import fiducial_msgs.msg
import assistiverobot_msgs.msg
import roslaunch


""" Global variable """
arm_joint_number = 0
finger_number = 0
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentFingerPosition = [0.0, 0.0, 0.0]
pub_base_cmd = rospy.Publisher("/base_cmd", assistiverobot_msgs.msg.Twist2DStamped, queue_size=1)

def allowence(x, allowed):
    if abs(x) <= allowed:
        return 0
    else:
        return x

def publishControl(Vx,Vy,W):
        base_cmd_msg = assistiverobot_msgs.msg.Twist2DStamped()
        base_cmd_msg.header.stamp = rospy.Time.now()
        base_cmd_msg.v_x =  Vx
        base_cmd_msg.v_y = Vy
        base_cmd_msg.omega = W

        pub_base_cmd.publish(base_cmd_msg)


def gripper_client(finger_positions):
    
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

struct Pose
    field double[] pos
    field double[] ori
end struct


object Kinova
    property uint8 camera_open

    # camera control functions
    function double[] getOri()
    function double[] getPos()
    function double[] getBasePos()
    function double[] getDistance()
    function void moveBase(double[] pos)
    function void startBase()
    function void stopBase()
    function double cartesian_pose_client(double[] pos, double[] ori, double relative)
    function void closeFinger(double[] values)
    
    function single cartesianPathTraj(Pose{list} waypoint)
    function double poseTargetTraj(double[] pos, double[] ori)
    function void execute(double wait)

    function void addBox(string name, double[] dim, double[] pos)
    function void removeScene(string name)
    function void attachBox(string jointName, string boxName)
    function void removeAttachedObject(string link, string name)

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
        self.base_pos = None # base position
        self.fix_pos = None # tag position
        self.distance = None
        self.moving = False
        self.start_base = None
        # self.startBase()
        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        '''
        self._plan = None
        self.arm = moveit_commander.MoveGroupCommander('arm')

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        # self.arm.setEndEffectorLink('j2n6s300_end_effector')
        rospy.sleep(3)
        '''
#         # self.arm.
#         self.arm.set_planning_time(60)
#         self.arm.set_num_planning_attempts(30)
#         # self.arm.set_joint_value_target( [0.0, 3.1415926535885, 3.1415926535895, 0.0, 0.0, 0.0] )
        
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position.x = -0.0788955718279 # self._ee_pos[0] 
        # pose_goal.position.y = -0.628492593765 # self._ee_pos[1]
        # pose_goal.position.z = 0.464324653149 # self._ee_pos[2] - 0.05
        # pose_goal.orientation.w= 0.946 # self._ee_ori[0]
        # pose_goal.orientation.x = 0.035 # self._ee_ori[1]
        # pose_goal.orientation.y = 0.316 # self._ee_ori[2]
        # pose_goal.orientation.z = -0.064 # self._ee_ori[3]
#         print self._ee_ori
# #  [-0.0788955718279, -0.628492593765, 0.464324653149]
# # Cartesian orientation in Quaternion is: 
# # qx 0.035, qy 0.316, qz -0.064, qw 0.946

#         self.arm.set_pose_target(pose_goal)
#         self._plan = self.arm.plan()


#         plan2 = self.arm.plan()
#         self.arm.execute(plan2, wait=True)
#         robot = moveit_commander.RobotCommander()
        # print robot.get_current_state()
        # print robot.get_group_names()
        # ['arm', 'gripper']



        # image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, process) 

    def close(self):
        # self._running = False
        # self._t_joints.join()
        self._t_effector.join()
        # self._t_command.join()

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
    

    '''
    moving base functionality

    '''    
    def getBasePos(self):
        return self.base_pos

    def getDistance(self):
        return self.distance


    def moveBase(self, pos):
        self.moving = True
        self.base_goal = np.array(pos) + self.distance

    def startBase(self):
        rospy.sleep(20)
        if self.start_base == None:
            package = 'goal_base_tf2' 
            executable = 'goal_base_listener_node.py' 
            node = roslaunch.core.Node(package, executable)
            launch = roslaunch.scriptapi.ROSLaunch() 
            launch.start()
            print 'start'
            self.start_base = launch.launch(node)
        elif not self.start_base.is_alive():
            self.start_base.start()
            rospy.sleep(1)

    
    def stopBase(self):
        # print process.is_alive() 
        if self.start_base.is_alive():
            self.start_base.stop()
            rospy.sleep(1)


    '''
    end moving base

    '''
    def execute(self, wait):
        if int(wait) == 1:
            wait = True
        else:
            wait = False
        self.arm.execute(self._plan, wait=True)

    def cartesianPathTraj(self, points):
        # try:
        #     print value[0].pos
        # except:
        #     try:
        #         print value.pos
        #     except:
        #         print "not list type"
        poses = []
        for pose in points:
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = pose.pos[0]
            pose_goal.position.y = pose.pos[1]
            pose_goal.position.z = pose.pos[2]
            pose_goal.orientation.x = pose.ori[0]
            pose_goal.orientation.y = pose.ori[1]
            pose_goal.orientation.z = pose.ori[2]
            pose_goal.orientation.w = pose.ori[3]
            poses.append(pose_goal)

        # add initialize into the waypoints
        # if len(poses) == 1:
        #     if self._valid_limb_names[limb] == 'left':
        #         pose1 = self.left_arm.get_current_pose().pose

        #     elif self._valid_limb_names[limb] == 'right':
        #         pose1 = self.right_arm.get_current_pose().pose

        #     poses = [pose1, poses[0]]
        print poses
        # pose1 = self.right_arm.get_current_pose().pose
        # pose2 = copy.deepcopy(pose1)
        # pose2.position.y += 0.15
        # print pose1
        # print pose2
        # waypoints = [pose1, pose2]
        (self._plan, fraction) = self.arm.compute_cartesian_path( poses,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
        # print plan.joint_trajectory.points[0].positions
        return fraction


    def poseTargetTraj(self, pos, ori):
        # end_effector = self.left_arm.get_end_effector_link()
        # wpose = self.left_arm.get_current_pose(end_effector).pose
        # print wpose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]
        pose_goal.orientation.x= ori[0]
        pose_goal.orientation.y = ori[1]
        pose_goal.orientation.z = ori[2]
        pose_goal.orientation.w = ori[3]


        self.arm.set_pose_target(pose_goal)
        self._plan = self.arm.plan()
        return len(self._plan.joint_trajectory.points)

    ## @brief if two scenes share same name, the second one will replace the first one
    ## @param name: name of the scene
    ## @param dim: one by three array, dimension of the scene
    ## @param pos: one by three array, position of the scene
    def addBox(self, name, dim, pos):
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        self.scene.add_box(name, p, dim)


    ## @param name: the name of the scene will be removed
    def removeScene(self, name):
        self.p.removeCollisionObject(name)


    ## @brief attach the box onto one specific joint
    ## @param name: name of the scene
    ## @param dim: one by three array, dimension of the scene
    ## @param pos: one by three array, position of the scene
    def attachBox(self, jointName, boxName):
        self.scene.attach_box(jointName, boxName)

    def removeAttachedObject(self, link, name=None):
        print name
        if name == '' or name == "1":

            name = None
        self.scene.remove_attached_object(link, name)



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
        i = 0
        #while i < 3 and not self.closeGoal(position):
        if 1:
            i += 1
            goal = kinova_msgs.msg.ArmPoseGoal()
            goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
            goal.pose.pose.position = geometry_msgs.msg.Point(
                x=position[0], y=position[1], z=position[2])
            goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
                x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

            client.send_goal(goal)

            if client.wait_for_result(rospy.Duration(10.0)):
                pass
                # print client.get_result()
                #return 1
                #return client.get_result()
            else:
                client.cancel_all_goals()
                print('        the cartesian action timed-out')
                #return 0
                #return None
            # print "Approaching the goal"
        return 1

    def closeGoal(self, position):
        position = np.array(position)
        return np.prod(np.abs(position - self._ee_pos) < 0.1)

    def closeFinger(self, values):
        #print('Sending finger position ...')
        result = gripper_client(values)
        #print('Finger position sent!')

    def handle_goal_base_pose(self, msg):
        # msg.transforms # Type: array of fiducial_msgs.msg.FiducialTransform

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        fix_tag = None
        base_tag = None
        for detected_transform in msg.transforms:
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "kin1_rgb_optical_frame"
            id1 = detected_transform.fiducial_id
            if id1 == 35:
                self.base_pos = np.array([detected_transform.transform.translation.x, detected_transform.transform.translation.y])
            if id1 == 15:
                # print detected_transform.translation.x
                self.fix_pos = np.array([detected_transform.transform.translation.x, detected_transform.transform.translation.y])
        self.distance = self.base_pos - self.fix_pos
        
        # move the base if needed
        if not self.moving:
            return 
        
        # Control Law Gains
        v_p_gain = 0.364425 *0.75 # Speed is max when error is larger than 1meter
        v_d_gain = 0.0
        w_p_gain = 0.848960 *0.5# Angular speed is when error is larger than 1 radian 
        w_d_gain = 0.0

        # marker id's
        base_id = 35
        goal1_id = 15
        goal2_id = 15
        
        P_bg = self.base_goal - self.distance
            # Control Law
        P_bg = [allowence(n, 0.005) for n in P_bg] # 5cm
        # E_bg = [allowence(n, 0.015) for n in E_bg] # 5 degrees = 0.09 radians

        V_x = P_bg[0] * v_p_gain
        V_y = P_bg[1] * v_p_gain
        
        # no rotation now
        # W_z = E_bg[2] * w_p_gain
        W_z = 0
        # Create Velocity Message to move end effector to Goal
        publishControl( V_x, -V_y, W_z)
        # add by Ruijie
        # rospy.sleep(1) 
        print (V_x, V_y, W_z)
        rospy.sleep(0.3)
        publishControl(0,0,0)
        # Check whether the goal position is reached
        if  (abs(V_x) <= 0.01 and abs(V_y) <= 0.01 and abs(W_z) <= 0.03):
            self.moving = False

    # worker function to request and update end effector data for baxter
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, self.process) 
        rospy.Subscriber('/fiducial_transforms', fiducial_msgs.msg.FiducialTransformArray, self.handle_goal_base_pose)
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

    kinova_obj.close()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
    # export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH
