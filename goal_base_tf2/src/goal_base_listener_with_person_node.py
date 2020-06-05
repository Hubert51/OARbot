#! /usr/bin/env python
import rospy

import math
import tf2_ros

import std_msgs.msg
import geometry_msgs.msg
# Because of transformations
import tf_conversions

import assistiverobot_msgs.msg 

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

if __name__ == '__main__':
    rospy.init_node('tf2_goal_base_listener_with_person')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
   
    # Publisher
    pub_base_cmd = rospy.Publisher("/base_cmd", assistiverobot_msgs.msg.Twist2DStamped, queue_size=1)
     
    # Control Law Gains
    v_p_gain = 0.364425 *0.75 # Speed is max when error is larger than 1meter
    v_d_gain = 0.0
    w_p_gain = 0.848960 *0.5# Angular speed is when error is larger than 1 radian 
    w_d_gain = 0.0
    
    # marker id's
    base_id = 25
    goal1_id = 15
    
    mode = 1 # 1: Reach to goal1, 2: Reach to goal2

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            T_base2goal1 = tfBuffer.lookup_transform('goal_base_id_{}_frame'.format(base_id), 'goal_base_id_{}_added_frame'.format(goal1_id), rospy.Time())
            T_base2goal2 = tfBuffer.lookup_transform('goal_base_id_{}_frame'.format(base_id), 'goal_base_person_added_frame', rospy.Time()) 
            
            T_goal1tobase = tfBuffer.lookup_transform('goal_base_id_{}_added_frame'.format(goal1_id), 'goal_base_id_{}_frame'.format(base_id), rospy.Time())
            T_goal2tobase = tfBuffer.lookup_transform('goal_base_person_added_frame', 'goal_base_id_{}_frame'.format(base_id), rospy.Time())
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            publishControl(0, 0, 0)
            continue
            
        if (mode == 1):
            # Set Position Vector Base(b) to Goal1(g) wrt Base(b).
            x_b = T_base2goal1.transform.translation.x 
            y_b = T_base2goal1.transform.translation.y  
            z_b = T_base2goal1.transform.translation.z
            
            P_bg = [x_b, y_b, z_b]
            print (P_bg)
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
            publishControl(V_y, -V_x, W_z)
            
            # print (V_x, V_y, W_z)
            # Check whether the goal position is reached
            if  (abs(V_x) <= 0.01 and abs(V_y) <= 0.01 and abs(W_z) <= 0.02):
                mode = 2 # Set mode 2
                inp = raw_input("Mode 1 Done, Press Enter to continue, 'e' to exit...")
                if inp == 'e':
                    break
                
        if (mode == 2):
            # Set Position Vector Base(b) to Goal1(g) wrt Base(b).
            x_b = T_base2goal2.transform.translation.x 
            y_b = T_base2goal2.transform.translation.y  
            z_b = T_base2goal2.transform.translation.z
            
            P_bg = [x_b, y_b, z_b]
            print (P_bg)
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
            #W_z = 0.0       
                   
            # Create Velocity Message to move end effector to Goal
            publishControl(V_y, -V_x, W_z)
            
            # print (V_x, V_y, W_z)
            # Check whether the goal position is reached
            if  (V_x <= 0.01 and V_y <= 0.01 and W_z <= 0.02):
                mode = 1 # Set mode 2
                inp = raw_input("Mode 2 Done, Press Enter to continue, 'e' to exit...")
                if inp == 'e':
                    break
                
                
                
                
        rate.sleep()
