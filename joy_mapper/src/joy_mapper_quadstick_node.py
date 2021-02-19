#!/usr/bin/env python
import rospy
import numpy as np
import math

from assistiverobot_msgs.msg import Twist2DStamped, StepCmdStamped
from sensor_msgs.msg import Joy
from kinova_msgs.msg import PoseVelocity, FingerPosition
from kinova_msgs.srv import HomeArm
import kinova_msgs.msg

import actionlib

from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.joy = None
        self.currentFingerPosition = [0.0, 0.0, 0.0]    
        
        self.action_address = '/j2n6s300_driver/fingers_action/finger_positions'
        self.client = actionlib.SimpleActionClient(self.action_address, kinova_msgs.msg.SetFingersPositionAction)
        self.goal = kinova_msgs.msg.SetFingersPositionGoal()
    

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.364425)
        self.w_gain = self.setupParam("~omega_gain", 0.848960)
        self.v_lin_gain = self.setupParam("~linear_vel_gain",0.2)
        self.w_ang_gain = self.setupParam("~angular_vel_gain",1.07)
        self.finger_maxTurn = self.setupParam("~finger_maxTurn",7300)  # max thread rotation for one finger
        self.finger_move_gain = self.setupParam("~finger_move_gain",200) # Max Increment in position of finger per command 

        # Publications
        self.pub_base_cmd = rospy.Publisher("~base_cmd", Twist2DStamped, queue_size=1)
        self.pub_step_cmd = rospy.Publisher("~step_cmd", StepCmdStamped, queue_size=1)
        self.pub_pose_vel_cmd = rospy.Publisher("~pose_vel_cmd", PoseVelocity, queue_size=1)
        
        
        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_finger_pos_ = rospy.Subscriber("j2n6s300_driver/out/finger_position", FingerPosition, self.setCurrentFingerPosition, queue_size=1)
        

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        # if left and center is sipped, control arm angular velocity
        if self.joy.buttons[10] == 1:
            self.publishArmAngControl()
            # a = 1
        
        # if right and center is sipped, control fingers
        elif self.joy.buttons[11] == 1:
            self.publishFingerControl()
            # a = 1
            
        # if only center is sipped, control arm linear velocity
        elif self.joy.buttons[0] == 1:
            self.publishArmLinControl()
            
            
        # If right is puffed, move the robot to home position
        elif self.joy.buttons[7] == 1:
            self.homeArm()
            
        # Else, control Base, 
        else:
            # self.publishArmLinControl()
            self.publishControl()
            self.publishControlStep()
            # a = 1
            
            
        

    def publishControl(self):
        base_cmd_msg = Twist2DStamped()
        base_cmd_msg.header.stamp = self.joy.header.stamp
        base_cmd_msg.v_x = self.joy.axes[1] * self.v_gain * 0.5 # Left\Right
        if self.joy.buttons[1] == 0: # if lip is not pressed
            base_cmd_msg.v_y = self.joy.axes[0] * self.v_gain * 0.5# Forward\Backward
        elif self.joy.buttons[1] == 1: # if lip is pressed    
            base_cmd_msg.omega = self.joy.axes[0] * self.w_gain * 0.5 # Rotate Left/Right with forward and backward
        
        self.pub_base_cmd.publish(base_cmd_msg)
        
    def publishControlStep(self):
        step_cmd_msg = StepCmdStamped()
        step_cmd_msg.header.stamp = self.joy.header.stamp
        if (self.joy.buttons[6] == 1): # left puffed
            step_cmd_msg.drive = +1 # go up
        elif (self.joy.buttons[4] == 1): # left sipped
            step_cmd_msg.drive = -1 # go down
        else:
            step_cmd_msg.drive = 0 # stop
        
        self.pub_step_cmd.publish(step_cmd_msg)
    
    def publishArmLinControl(self):
        pose_vel_msg = PoseVelocity()
        pose_vel_msg.twist_linear_x = self.joy.axes[1] * self.v_lin_gain  # Left\Right
        
        if self.joy.buttons[1] == 0: # if lip is not pressed
            pose_vel_msg.twist_linear_y = self.joy.axes[0] * self.v_lin_gain # Forward\Backward
        elif self.joy.buttons[1] == 1: # if lip is pressed    
            pose_vel_msg.twist_linear_z = self.joy.axes[0] * self.v_lin_gain   # Up/Down Axis stick right
            
        pose_vel_msg.twist_angular_x = 0
        pose_vel_msg.twist_angular_y = 0
        pose_vel_msg.twist_angular_z = 0
        
        self.pub_pose_vel_cmd.publish(pose_vel_msg)
        
    def publishArmAngControl(self):
        pose_vel_msg = PoseVelocity()
        pose_vel_msg.twist_linear_x = 0
        pose_vel_msg.twist_linear_y = 0
        pose_vel_msg.twist_linear_z = 0
        
        pose_vel_msg.twist_angular_x = self.joy.axes[1] * self.w_ang_gain # Up/Down Axis stick left
        
        if self.joy.buttons[1] == 0: # if lip is not pressed
            pose_vel_msg.twist_angular_y = self.joy.axes[0] * self.w_ang_gain # Left/Right Axis stick left
        elif self.joy.buttons[1] == 1: # if lip is pressed    
            pose_vel_msg.twist_angular_z = self.joy.axes[0] * -self.w_ang_gain # Left/Right Axis stick right      
        
        self.pub_pose_vel_cmd.publish(pose_vel_msg)
        
    def publishFingerControl(self):
        try:
            # Set goal positions        
            positions_temp1 = [max(0.0, n + self.joy.axes[1]*self.finger_move_gain) for n in self.currentFingerPosition]
            positions_temp1[:-1] = [max(0.0, n + self.joy.buttons[1]*self.joy.axes[1]*self.finger_move_gain) for n in self.currentFingerPosition[:-1]]
            positions_temp2 = [min(n, self.finger_maxTurn) for n in positions_temp1]
            positions = [float(n) for n in positions_temp2]
            
            result = self.gripper_client(positions)
            
        except rospy.ROSInterruptException:
            print('program interrupted before completion')
        
    def setCurrentFingerPosition(self, feedback):
        self.currentFingerPosition[0] = feedback.finger1
        self.currentFingerPosition[1] = feedback.finger2
        self.currentFingerPosition[2] = feedback.finger3
        
        
        
    def gripper_client(self, finger_positions):
        """Send a gripper goal to the action server."""
        
        self.client.wait_for_server()

        
        self.goal.fingers.finger1 = float(finger_positions[0])
        self.goal.fingers.finger2 = float(finger_positions[1])
        # The MICO arm has only two fingers, but the same action definition is used
        if len(finger_positions) < 3:
            self.goal.fingers.finger3 = 0.0
        else:
            self.goal.fingers.finger3 = float(finger_positions[2])
        self.client.send_goal(self.goal)
        if self.client.wait_for_result(rospy.Duration(5.0)):
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            rospy.WARN('        the gripper action timed-out')
            return None
            
    def homeArm(self):
        service_address = '/j2n6s300_driver/in/home_arm'
        rospy.wait_for_service(service_address)
        try:
            home = rospy.ServiceProxy(service_address, HomeArm)
            home()
            return None
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
        

        
# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10

if __name__ == "__main__":
    rospy.init_node("joy_mapper_quadstick",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
