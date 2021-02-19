#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf2_ros

import geometry_msgs.msg
import std_msgs.msg
import kinova_msgs.msg # JointAngles, KinovaPose
import tf_conversions

class ForwardKinematics(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.pose_msg = geometry_msgs.msg.PoseStamped()

        # Setup Parameters
        self.D1 = self.setupParam("~D1", 0.2755) # Distance Base to Shoulder
        self.D2 = self.setupParam("~D2", 0.4100) # Upper arm Length (shoulder to elbow)
        self.D3 = self.setupParam("~D3", 0.2073) # Forearm Length (Elbow to wrist)
        self.D4 = self.setupParam("~D4", 0.0741) # First wrist length (center of actuator 4 to center of actuator 5)
        self.D5 = self.setupParam("~D5", 0.0741) # Second wrist length (center of actuator 5 to center of actuator 6)
        self.D6 = self.setupParam("~D6", 0.1600) # Wrist to center of the hand
        self.e2 = self.setupParam("~e2", 0.0098) # Joint 3-4 lateral offset

        self.aa = math.radians(30)
        self.sa = math.sin(self.aa)
        
        self.s2a = math.sin(2*self.aa) # Sine of angle of curvature of wrist segment
        self.d4b = self.D3 + (self.sa/self.s2a)*self.D4 # Length of straight-line segment from elbow to end of first sub-segment of first wrist segment.
        self.d5b = (self.sa/self.s2a)*(self.D4+self.D5) # Length of straight-line segment consisting of second sub-segment of first wrist segment and first sub-segment of second wrist segment
        self.d6b = (self.sa/self.s2a)*self.D5 + self.D6 # Length of straight-line segment consisting of second sub-segment of second wrist segment and distance from wrist to the center of the hand

        # Publications
        self.pub_tool_pose = rospy.Publisher("~DH_tool_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
        
        
        # Subscriptions
        self.sub_joint_positions_ = rospy.Subscriber("j2n6s300_driver/out/joint_angles", kinova_msgs.msg.JointAngles, self.calculateForwardKinematics, queue_size=1)
        

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def calculateForwardKinematics(self, joint_angles_msg):
        self.joint_angles = joint_angles_msg
        self.q1 = self.joint_angles.joint1      
        self.q2 = self.joint_angles.joint2
        self.q3 = self.joint_angles.joint3
        self.q4 = self.joint_angles.joint4
        self.q5 = self.joint_angles.joint5
        self.q6 = self.joint_angles.joint6

        # print (self.q1,self.q2,self.q3,self.q4,self.q5,self.q6)
         
        self.calculateToolPose()
            
            
        

    def calculateToolPose(self):
        self.T01 = self.DH2T(-90, 0, self.D1, -(self.q1-180))
        self.T12 = self.DH2T(180, self.D2, 0,  self.q2-270)
        self.T23 = self.DH2T(-90, 0, -self.e2, self.q3-90)
        self.T34 = self.DH2T( 60, 0, -self.d4b, self.q4-180)
        self.T45 = self.DH2T( 60, 0, -self.d5b, self.q5-180)
        self.T56 = self.DH2T(180, 0, -self.d6b, self.q6+90)
        
        self.T02 = self.T01.dot(self.T12)
        self.T03 = self.T02.dot(self.T23)
        self.T04 = self.T03.dot(self.T34)
        self.T05 = self.T04.dot(self.T45)
        self.T06 = self.T05.dot(self.T56)
        
        #print(self.d4b)
        #print (self.T04)
        
        # Rotation matrix and Position Vector of end effector
        self.R06 = self.T06[:3,:3] #3x3
        self.P06 = self.T06[:3,3] #1x3
        
        # Convert Rotation matrix to quaternion
        w = 0.5 * math.sqrt(1.0 + np.matrix.trace(self.R06))
        x = (self.R06[2,1]-self.R06[1,2])/(4*w)
        y = (self.R06[0,2]-self.R06[2,0])/(4*w)
        z = (self.R06[1,0]-self.R06[0,1])/(4*w)
        
        self.quat = [x,y,z,w]
        
        self.pose_msg.header = std_msgs.msg.Header(frame_id='j2n6s300_link_base')
        self.pose_msg.pose.position = geometry_msgs.msg.Point( x=self.P06[0], y=self.P06[1], z=self.P06[2])
        self.pose_msg.pose.orientation = geometry_msgs.msg.Quaternion( x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        
        self.pub_tool_pose.publish(self.pose_msg)
        
        """
        # Euler XYZ
        self.euler_xyz = tf_conversions.transformations.euler_from_quaternion(self.quat)
        
        # Publish Pose message
        self.pose_msg.X = self.P06[0]
        self.pose_msg.Y = self.P06[1]
        self.pose_msg.Z = self.P06[2]
        self.pose_msg.ThetaX = self.euler_xyz[0]
        self.pose_msg.ThetaY = self.euler_xyz[1]
        self.pose_msg.ThetaZ = self.euler_xyz[2]
        
        self.pub_tool_pose.publish(self.pose_msg) 
        """
        

    def DH2T(self, A, a, d, Q):
        A = math.radians(A)
        Q = math.radians(Q)
        
        T = np.zeros((4,4))
        T[0,:] = [math.cos(Q), -math.cos(A)*math.sin(Q),   math.sin(A)*math.sin(Q), a*math.cos(Q)]
        T[1,:] = [math.sin(Q),  math.cos(A)*math.cos(Q),  -math.sin(A)*math.cos(Q), a*math.sin(Q)]
        T[2,:] = [0,            math.sin(A),               math.cos(A),             d            ]
        T[3,:] = [0,            0,                         0,                       1            ]
        
        return T
        
    

if __name__ == "__main__":
    rospy.init_node("Arm_DH_Forward_Kin",anonymous=False)
    Arm_DH_Forward_Kin = ForwardKinematics()
    rospy.spin()
