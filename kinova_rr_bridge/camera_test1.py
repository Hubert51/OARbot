#!/usr/bin/env python
import roslib
# roslib.load_manifest('baxter_rr_bridge')
from cv_bridge import CvBridge, CvBridgeError

import rospy
import baxter_interface
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import time

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback
import cv2
import cv2.aruco as aruco

bridge = CvBridge()
cv2_img = None


def set_imagedata(self,camdata):
    print("Received an image!")
    global cv2_img
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")


if __name__ == '__main__':
    rospy.init_node('baxter_cameras', anonymous = True)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.set_imagedata) 

    # The following function is to set camera parameters manually
    while 1:
        if cv2_img is not None:
            # process_img(cv2_img)
            cv2.imshow('img', cv2_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(.2)
    cv2.destroyAllWindows()

