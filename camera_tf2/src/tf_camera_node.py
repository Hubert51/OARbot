#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
# import kinova_msgs.msg


def handle_camera_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "j2n6s300_end_effector"
    t.child_frame_id = "camera_link"
    # t.child_frame_id = "camera_color_optical_frame"
    t.transform.translation.x = 0.0175
    t.transform.translation.y = 0.038 + 0.04638
    t.transform.translation.z = -1*(0.16 - 0.110) #-1*(0.16 - 0.04)  

    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = 0.5# q[0]
    t.transform.rotation.y = 0.5# q[1]
    t.transform.rotation.z = 0.5# q[2]
    t.transform.rotation.w = -0.5# q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_camera_broadcaster')
    rospy.Subscriber('/j2n6s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, handle_camera_pose)
    rospy.spin()
