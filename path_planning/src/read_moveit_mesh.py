#! /usr/bin/python2

import pybullet as p
import time
import pybullet_data

import numpy as np
from math import *

import threading

import os, sys
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_msgs
from moveit_msgs.msg import PlanningScene
import geometry_msgs
import sensor_msgs
from sensor_msgs.msg import JointState
import tf2_ros
import tf
from math import pi


def readPlanningSceneWorld(msg):
    print msg


if __name__ == '__main__':
    rospy.init_node('read_mesh', anonymous = True)
    topic_address = "/move_group/monitored_planning_scene"
    rospy.Subscriber(topic_address, PlanningScene, readPlanningSceneWorld)
    rospy.wait_for_message(topic_address, PlanningScene)
    print 'position listener obtained message for joint position. '