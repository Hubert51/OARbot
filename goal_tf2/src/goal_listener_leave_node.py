#! /usr/bin/env python
import rospy

import math
import tf2_ros

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
# Because of transformations
import tf_conversions

def allowence(x, allowed):
    if abs(x) <= allowed:
        return 0
    else:
        return x
        
def publishControl(V_x, V_y, V_z, W_x, W_y, W_z):
    pose_vel_msg = kinova_msgs.msg.PoseVelocity()
    
    pose_vel_msg.twist_linear_x = V_x  
    pose_vel_msg.twist_linear_y = V_y
    pose_vel_msg.twist_linear_z = V_z
    pose_vel_msg.twist_angular_x = W_x
    pose_vel_msg.twist_angular_y = W_y
    pose_vel_msg.twist_angular_z = W_z

    pub_pose_vel_cmd.publish(pose_vel_msg)


if __name__ == '__main__':
    rospy.init_node('tf2_goal_listener_leave')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    
    
    # Publisher
    pub_pose_vel_cmd = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity, queue_size=1)
    # Control Law Gains
    v_p_gain = 2*1.5
    v_d_gain = 0.0
    w_p_gain = 1.362366*1.2
    w_d_gain = 0.0
    
    mode = 1 # 1: Reach to goal, 2: Open Fingers, 3: Move to Goal_after

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            T_base2goal = tfBuffer.lookup_transform('j2n6s300_link_base', 'goal_frame', rospy.Time())
            T_base2end = tfBuffer.lookup_transform('j2n6s300_link_base', 'j2n6s300_end_effector', rospy.Time())
            T_end2goal = tfBuffer.lookup_transform('j2n6s300_end_effector', 'goal_frame', rospy.Time())
            
            T_base2goalafter = tfBuffer.lookup_transform('j2n6s300_link_base', 'goal_after_frame', rospy.Time())
            T_end2goalafter = tfBuffer.lookup_transform('j2n6s300_end_effector', 'goal_after_frame', rospy.Time())
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            publishControl(0, 0, 0, 0, 0, 0)
            continue
            
        
        if (mode == 1):
            # Find Position Vector End-Effector(e) to Goal(g) wrt Base Link Frame(b).
            x_b = T_base2goal.transform.translation.x - T_base2end.transform.translation.x
            y_b = T_base2goal.transform.translation.y - T_base2end.transform.translation.y
            z_b = T_base2goal.transform.translation.z - T_base2end.transform.translation.z
            
            P_e2g_b = [x_b, y_b, z_b]
            
            # Find Quaternion(Q) and XYZ Euler(E) Rotation between End-effector(e) to Goal(g)
            # Quaternion
            Q_eg = [T_end2goal.transform.rotation.x, T_end2goal.transform.rotation.y, T_end2goal.transform.rotation.z, T_end2goal.transform.rotation.w]
            # Euler XYZ
            E_e2g = tf_conversions.transformations.euler_from_quaternion(Q_eg)
            
            # Control Law
            P_e2g_b = [allowence(n, 0.005) for n in P_e2g_b] # 0.5cm
            E_e2g = [allowence(n, 0.015) for n in E_e2g] # 5 degrees = 0.09 radians
            
            V_x = P_e2g_b[0] * v_p_gain
            V_y = P_e2g_b[1] * v_p_gain
            V_z = P_e2g_b[2] * v_p_gain
            
            W_x = E_e2g[0] * w_p_gain
            W_y = E_e2g[1] * w_p_gain
            W_z = E_e2g[2] * w_p_gain
                   
            # Create Velocity Message to move end effector to Goal
            publishControl(V_x, V_y, V_z, W_x, W_y, W_z)
            
            # print (V_x, V_y, V_z, W_x, W_y, W_z)
            # Check whether the goal position is reached
            if  (V_x <= 0.02 and V_y <= 0.02 and V_z <= 0.02 and W_x <= 0.1 and W_y <= 0.1 and W_z <= 0.1):
                mode = 2 # Set mode 2
        
        # If goal is reached, close the fingers to hold the object
        if (mode == 2):
            action_address = '/j2n6s300_driver/fingers_action/finger_positions'
            client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
            finger_goal = kinova_msgs.msg.SetFingersPositionGoal()
            
            """Send a gripper goal to the action server."""
        
            client.wait_for_server()
            
            finger_goal.fingers.finger1 = 0.0
            finger_goal.fingers.finger2 = 0.0
            finger_goal.fingers.finger3 = 0.0
            
            client.send_goal(finger_goal)
            if client.wait_for_result(rospy.Duration(5.0)):
                finger_result = client.get_result()
                mode = 3
            else:
                client.cancel_all_goals()
                rospy.WARN('        the gripper action timed-out')
        
        # After fingers are opened, move to goal_after
        if (mode == 3):
            # Find Position Vector End-Effector(e) to Goal(g) wrt Base Link Frame(b).
            x_b = T_base2goalafter.transform.translation.x - T_base2end.transform.translation.x
            y_b = T_base2goalafter.transform.translation.y - T_base2end.transform.translation.y
            z_b = T_base2goalafter.transform.translation.z - T_base2end.transform.translation.z
            
            P_e2g_b = [x_b, y_b, z_b]
            
            # Find Quaternion(Q) and XYZ Euler(E) Rotation between End-effector(e) to Goal(g)
            # Quaternion
            Q_eg = [T_end2goalafter.transform.rotation.x, T_end2goalafter.transform.rotation.y, T_end2goalafter.transform.rotation.z, T_end2goalafter.transform.rotation.w]
            # Euler XYZ
            E_e2g = tf_conversions.transformations.euler_from_quaternion(Q_eg)
            
            # Control Law
            P_e2g_b = [allowence(n, 0.005) for n in P_e2g_b] # 0.5cm
            E_e2g = [allowence(n, 0.015) for n in E_e2g] # 5 degrees = 0.09 radians
            
            V_x = P_e2g_b[0] * v_p_gain
            V_y = P_e2g_b[1] * v_p_gain
            V_z = P_e2g_b[2] * v_p_gain
            
            W_x = E_e2g[0] * w_p_gain
            W_y = E_e2g[1] * w_p_gain
            W_z = E_e2g[2] * w_p_gain
                   
            # Create Velocity Message to move end effector to Goal
            publishControl(V_x, V_y, V_z, W_x, W_y, W_z)
            
            # Check whether the goal position is reached
            if  (V_x <= 0.02 and V_y <= 0.02 and V_z <= 0.02 and W_x <= 0.1 and W_y <= 0.1 and W_z <= 0.1):
                print('Successful...')
                break
          
        
              
        rate.sleep()
    
    
    
    
        
        
        
        
