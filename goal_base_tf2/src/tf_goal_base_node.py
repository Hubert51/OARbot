#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
# import kinova_msgs.msg
import fiducial_msgs.msg


def handle_goal_base_pose(msg):
    # msg.transforms # Type: array of fiducial_msgs.msg.FiducialTransform

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    
    for detected_transform in msg.transforms:
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "kin1_rgb_optical_frame"
        t.child_frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
        t.transform = detected_transform.transform
        br.sendTransform(t)
        
        if detected_transform.fiducial_id == 15:
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
            t.child_frame_id = "goal_base_id_{}_added_frame".format(detected_transform.fiducial_id)
            
            # t.transform.translation.x = 0.750
            # t.transform.translation.y = 0.0
            t.transform.translation.x = -0.80
            t.transform.translation.y = 0.51 
            t.transform.translation.z = 0.0 

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
            
        if detected_transform.fiducial_id == 35:
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
            t.child_frame_id = "goal_base_id_{}_added_frame".format(detected_transform.fiducial_id)
            

            t.transform.translation.x = -0.2 # -0.40                               
            t.transform.translation.y = 0.7 # 0.0 
            t.transform.translation.z = 0.0 

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
        
   

if __name__ == '__main__':
    rospy.init_node('tf2_goal_base_broadcaster')
    rospy.Subscriber('/fiducial_transforms', fiducial_msgs.msg.FiducialTransformArray, handle_goal_base_pose)
    rospy.spin()
