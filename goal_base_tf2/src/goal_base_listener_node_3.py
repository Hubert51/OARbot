#! /usr/bin/env python
import rospy

import math
import tf2_ros

import std_msgs.msg
import geometry_msgs.msg
# Because of transformations
import tf_conversions

import assistiverobot_msgs.msg
import random

from __builtin__ import True



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
        
def cbModes(mode_msg):
    mode = mode_msg.data
    print (mode)
    print('MODE ', mode_msg)
    while not (mode == '-1'):
        try:
            T_base2goal1 = tfBuffer.lookup_transform('j2n6s300_link_base', 'goal_fridge', rospy.Time())
            T_base2goal2 = tfBuffer.lookup_transform('j2n6s300_link_base', 'goal_chair', rospy.Time())
            
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rate.sleep()
            publishControl(0, 0, 0)
            continue
            
        if (mode == '1'):
            # Set Position Vector Base(b) to Goal1(g) wrt Base(b).
            x_b = T_base2goal1.transform.translation.x 
            y_b = T_base2goal1.transform.translation.y  
            z_b = T_base2goal1.transform.translation.z
            
            P_bg = [x_b, y_b, z_b]
            # print (P_bg)
            # Find Quaternion(Q) and XYZ Euler(E) Rotation between base(e) to Goal(g)
            # Quaternion
            Q_bg = [T_base2goal1.transform.rotation.x, T_base2goal1.transform.rotation.y, T_base2goal1.transform.rotation.z, T_base2goal1.transform.rotation.w]
            # Euler XYZ
            E_bg = tf_conversions.transformations.euler_from_quaternion(Q_bg)
            
            # Control Law
            P_bg = [allowence(n, 0.05) for n in P_bg] # 5cm
            E_bg = [allowence(n, 0.015) for n in E_bg] # 5 degrees = 0.09 radians
            
            V_x = P_bg[0] * v_p_gain
            V_y = P_bg[1] * v_p_gain
            
            W_z = E_bg[2] * w_p_gain
            #W_z = 0.0       
                   
            # Create Velocity Message to move end effector to Goal
            publishControl(V_x, V_y, W_z)
            
            # print (V_x, V_y, W_z)
            # Check whether the goal position is reached
            if  (abs(V_x) <= 0.01 and abs(V_y) <= 0.01 and abs(W_z) <= 0.02):
                mode = '-1' # Set mode -1 to stop
                
        if (mode == '2'):
            # Set Position Vector Base(b) to Goal1(g) wrt Base(b).
            x_b = T_base2goal2.transform.translation.x 
            y_b = T_base2goal2.transform.translation.y  
            z_b = T_base2goal2.transform.translation.z
            
            P_bg = [x_b, y_b, z_b]
            # print (P_bg)
            # Find Quaternion(Q) and XYZ Euler(E) Rotation between base(e) to Goal(g)
            # Quaternion
            Q_bg = [T_base2goal2.transform.rotation.x, T_base2goal2.transform.rotation.y, T_base2goal2.transform.rotation.z, T_base2goal2.transform.rotation.w]
            # Euler XYZ
            E_bg = tf_conversions.transformations.euler_from_quaternion(Q_bg)
            
            # Control Law
            P_bg = [allowence(n, 0.05) for n in P_bg] # 5cm
            E_bg = [allowence(n, 0.015) for n in E_bg] # 5 degrees = 0.09 radians
            
            V_x = P_bg[0] * v_p_gain
            V_y = P_bg[1] * v_p_gain
            
            W_z = E_bg[2] * w_p_gain
            # print(E_bg[2])
            #W_z = 0.0       
                   
            # Create Velocity Message to move end effector to Goal
            publishControl(V_x, V_y, W_z) # V_y, -V_x, W_z)
            
            # print (V_x, V_y, W_z)
            # Check whether the goal position is reached
            if  (abs(V_x) <= 0.01 and abs(V_y) <= 0.01 and abs(W_z) <= 0.02):
                mode = '-1' # Set mode -1 to stop 

if __name__ == '__main__':
    rospy.init_node('tf2_goal_base_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
   
    # Publisher
    pub_base_cmd = rospy.Publisher("/base_cmd", assistiverobot_msgs.msg.Twist2DStamped, queue_size=1)
    
    
    # Control Law Gains
    v_p_gain = 0.364425 *0.75 # Speed is max when error is larger than 1meter
    v_d_gain = 0.0
    w_p_gain = 0.848960 *0.5 #0.5 # Angular speed is when error is larger than 1 radian 
    w_d_gain = 0.0
    
    mode = '0' # 1: Reach to goal1, 2: Reach to goal2
    
    # Subscriber
    sub_goal_mode = rospy.Subscriber("/goal_mode", std_msgs.msg.String, cbModes, queue_size=1) 
    
    rospy.spin()
