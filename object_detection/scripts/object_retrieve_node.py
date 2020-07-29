#! /usr/bin/python2

"""
1. wait for the user to input the object they want
2. do another detection to locate the object
3. publish the goal to the arm

"""



from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
bridge = CvBridge()

import cv2
import glob
import matplotlib.pyplot as plt
import pickle
import matplotlib.image as mpimg
import rospy
import time
import sys

cv2_img = None

import numpy as np
import os
# import pathlib
import six.moves.urllib as urllib
import sys
import tarfile
# import tensorflow as tf
import zipfile

import cob_object_detection_msgs.msg._DetectionArray
from std_msgs.msg import String
import message_filters
import threading

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt

import tf2_ros

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

# import pcl

INDEX = None
OBJECT_NAME = None
GRAB_ORI = [0.0761, 0.7057, 0.7032, -0.0410, ]
DETECT_ORI = [0.0068, 0.72, 0.69, 0.025]
# GRASP


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
    # to kinova action server
    client.send_goal(goal)

    # to user interface
    grasp_pose_pub.publish(goal.pose.pose)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

class RetriveServer(object):
    def __init__(self):
        self.index = None
        self.name = None
        self.detection = None
        self.bottle_shape = np.array([0.055, 0.175]) # the unit is meter
        self.pose = None
        self.upper_height = None
        self.lower_height = None

        self.run_grasp = False
        self.height_list = []

    # self._t_effector = threading.Thread(target=self.grasp_pose_detection)
        # self._t_effector.daemon = True
        # self._t_effector.start()

    def adjustArm(self, trans, bbox, depth):
        # adjust height
        depth = depth/1000
        ori = DETECT_ORI
        if bbox.y + bbox.height > 450:
            pass
            trans[2] -= 0.01
        elif bbox.y < 10:
            trans[2] += 0.01

        if depth > 0.15:
            trans[1] = 0.03
        ## the next action is grasping
        elif depth <= 0.15:
            trans[1] = 0.15
            trans[2] += 0.025
            ori = GRAB_ORI
            self.run_grasp = False

        return trans, ori



    def index_callback(self, msg):
        # "Store" message received.
        # print msg.data
        data = msg.data.split("-")
        self.index = int(data[1])
        self.name = data[0]
        self.grasp_pose_detection()

    def detection_callback(self, msg):
        self.detections = msg.detections

    def position_callback(self, msg):
        self.pose = msg.pose

    def fridge_callback(self, msg):
        self.height_list.append(msg.data[0])
        if len(self.height_list) >= 6:
            self.height_list.pop(0)
            ## if the height is almost the same
            if abs(max(self.height_list) - min(self.height_list)) < 0.1:
                self.lower_height = self.pose.position.z - np.mean(self.height_list)
                print "new lower height is {}, the current z axis is {}".format(self.lower_height, self.pose.position.z)



    def grasp_pose_detection(self):
        camera_shift_x = 0.045
        camera_shift_z = 0.05

        height_shift = 0.105
        self.run_grasp = True

        while self.run_grasp:
            while self.detections == None:
                pass
            detection = self.detections[self.index]
            assert detection.label == self.name

            bbox = detection.mask.roi
            # print bbox
            if bbox.x + bbox.width / 2 < 320:
                direction = 1
            else:
                direction = -1
            # print "direction is {}".format(direction)

            ## calculate the pixel length
            pixel_len = np.mean(self.bottle_shape / np.array([bbox.width, bbox.height]))
            # print "pixel length is {}".format(pixel_len)

            ## get the translation of arm
            img_center = np.array([640/2, 0, 480/2])
            obj_center = np.array([bbox.x + bbox.width/2, 0, bbox.y + bbox.height/2])
            result_shift = (obj_center-img_center) * pixel_len * np.array([1, 0, -1])
            trans = result_shift + np.array([-camera_shift_x, 0, camera_shift_z])
            trans, ori = self.adjustArm(trans, bbox, detection.depth)
            position = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
            position += trans
            temp_ori = self.pose.orientation
            orientations = [temp_ori.x, temp_ori.y, temp_ori.z, temp_ori.w]
            # print orientations

            cartesian_pose_client(position, ori)
            rospy.sleep(1)
            self.detections = None

        # print
        # print trans


if __name__ == '__main__':
    rospy.init_node('object_retrieve_node')

    # Publisher
    grasp_pose_pub = rospy.Publisher("/object_detection/grasp_pose", geometry_msgs.msg.Pose, queue_size=1)

    server = RetriveServer()
    # Subscriber
    index_sub = rospy.Subscriber("/selected_object_index", String, server.index_callback)
    detection_sub = rospy.Subscriber("/detected_object", cob_object_detection_msgs.msg.DetectionArray, server.detection_callback)
    rospy.Subscriber("/j2n6s300_driver/out/tool_pose", geometry_msgs.msg.PoseStamped, server.position_callback)
    rospy.Subscriber("/environment/fridge", std_msgs.msg.Float32MultiArray, server.fridge_callback)


rospy.spin()

