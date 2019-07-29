#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
# import kinova_msgs.msg


def handle_goal_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "marker_frame"
    t.child_frame_id = "goal_frame"
    # t.child_frame_id = "camera_color_optical_frame"
    t.transform.translation.x = - 0.15 # height
    t.transform.translation.y = 0.170 # distance
    t.transform.translation.z = 0

    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = -0.5# q[0]
    t.transform.rotation.y = -0.5# q[1]
    t.transform.rotation.z = 0.5# q[2]
    t.transform.rotation.w = -0.5# q[3]

    br.sendTransform(t)    
    
    ## AFTER GOAL FRAME
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "goal_frame"
    t.child_frame_id = "goal_after_frame"

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.15 # Up
    t.transform.translation.z = -0.15 # Back

    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_goal_broadcaster')
    rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, handle_goal_pose)
    rospy.spin()
