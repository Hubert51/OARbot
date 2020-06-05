
#! /usr/bin/env python
import rospy

import math
import tf2_ros

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/j2n6s300_driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=('j2n6s300_link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

if __name__ == '__main__':
    rospy.init_node('tf2_goal_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('j2n6s300_link_base', 'j2n6s300_end_effector', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
    
        position = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
        orientation = [trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
        
        try:
            print orientation
            
            result = cartesian_pose_client(position, orientation)

            print('Cartesian pose sent!')

        except rospy.ROSInterruptException:
            print "program interrupted before completion"
    
    