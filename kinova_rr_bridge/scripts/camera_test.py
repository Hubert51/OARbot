#!/usr/bin/env python2
import roslib
# roslib.load_manifest('baxter_rr_bridge')
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import time

import sys, argparse
import struct
import time
import RobotRaconteur as RR1
import thread
import threading
import numpy
import traceback
import cv2
import cv2.aruco as aruco

bridge = CvBridge()
cv2_img = None


def set_imagedata(msg):
    print("Received an image!")
    global cv2_img
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")


if __name__ == '__main__':
    rospy.init_node('baxter_cameras', anonymous = True)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, set_imagedata) 
    output = "3objects.mp4"
    while 1:
        if cv2_img is not None:
            height, width, channels = cv2_img.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use lower case
            out = cv2.VideoWriter(output, fourcc, 30.0, (width, height))
            break

    # The following function is to set camera parameters manually
    while 1:
        if cv2_img is not None:
            # process_img(cv2_img)
            cv2.imshow('img', cv2_img)
            out.write(cv2_img) # Write out frame to video



        if cv2.waitKey(1) == ord('q'):
            break

        # time.sleep(.2)
    out.release()
    cv2.destroyAllWindows()
