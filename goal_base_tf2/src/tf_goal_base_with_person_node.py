#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
# import kinova_msgs.msg
import assistiverobot_msgs.msg


def handle_goal_base_pose(msg):
    # msg.transforms # Type: array of fiducial_msgs.msg.FiducialTransform

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    
    for detected_point in msg.people_points:
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "kin1_rgb_optical_frame" # TODO: This is coming from IR frame but thats OK for the time being
        t.child_frame_id = "goal_base_person_frame"
        t.transform.translation.x = detected_point.point.x
        t.transform.translation.y = detected_point.point.y
        t.transform.translation.z = detected_point.point.z 

        q = tf_conversions.transformations.quaternion_from_euler(2.12, -0.49, 0.39) # TODO
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        
        # Calculate frame to be reached by robot as named "added" wrt persons frame
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "goal_base_person_frame"
        t.child_frame_id = "goal_base_person_added_frame"
        
        t.transform.translation.x = -0.80 # TODO
        t.transform.translation.y = -0.20 # TODO
        t.transform.translation.z = 0.0 

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0) # TODO
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
    
    
if __name__ == '__main__':
    rospy.init_node('tf2_goal_base_person_broadcaster')
    rospy.Subscriber('/people_points', assistiverobot_msgs.msg.user_points, handle_goal_base_pose) # TODO
    rospy.spin()
