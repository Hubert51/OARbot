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
        
        #if detected_transform.fiducial_id == 15: # ID 15: Ground Tag
        #    br = tf2_ros.TransformBroadcaster()
        #    t = geometry_msgs.msg.TransformStamped()
        #    t.header.stamp = rospy.Time.now()
        #    t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
        #    t.child_frame_id = "goal_ground".format(detected_transform.fiducial_id)
            
   
        #    t.transform.translation.x = 0.0
        #    t.transform.translation.y = 0.0 
        #    t.transform.translation.z = 0.0 

        #    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        #    t.transform.rotation.x = q[0]
        #    t.transform.rotation.y = q[1]
        #    t.transform.rotation.z = q[2]
        #    t.transform.rotation.w = q[3]

        #    br.sendTransform(t)
            
        if detected_transform.fiducial_id == 35: # ID 35: Fridge Tag
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
            t.child_frame_id = "goal_fridge"
            

            t.transform.translation.x = 0.63 # -0.40                               
            t.transform.translation.y = -0.35 # 0.0 
            t.transform.translation.z = 0.0 

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 1.57079633) # 90 degrees rotation
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
            
        if detected_transform.fiducial_id == 40: # ID 40: Microwave oven Tag
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
            t.child_frame_id = "goal_oven"
            

            t.transform.translation.x = 0.63                                
            t.transform.translation.y = -0.35  
            t.transform.translation.z = 0.0 

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 1.57079633*2) # 180 degrees rotation
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
            
        if detected_transform.fiducial_id == 25: # ID 25: Robot Base Tag
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
            t.child_frame_id = "j2n6s300_link_base"
            

            t.transform.translation.x = 0.0                                
            t.transform.translation.y = 0.40  
            t.transform.translation.z = 0.05 

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 1.57079633) # 90 degrees rotation
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
            
        if detected_transform.fiducial_id == 45: # ID 45: Person Chair Tag
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "goal_base_id_{}_frame".format(detected_transform.fiducial_id)
            t.child_frame_id = "goal_chair"
            

            t.transform.translation.x = -0.40                                
            t.transform.translation.y = -0.70  
            t.transform.translation.z = 0.00 

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 1.57079633) # 90 degrees rotation
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)
            
            
            
            
            
            
        
   

if __name__ == '__main__':
    rospy.init_node('tf2_goal_base_broadcaster')
    rospy.Subscriber('/fiducial_transforms', fiducial_msgs.msg.FiducialTransformArray, handle_goal_base_pose)
    rospy.spin()
