#!/usr/bin/env python2
import rospy
import numpy as np
import math
from math import pi
import tf
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_from_matrix, inverse_matrix, euler_from_quaternion

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import RobotRaconteur as RR

import sys, argparse
import thread
import threading
import time

import cv2
# import cv2.cv
import message_filters
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist
# from p2os_msgs.msg import MotorState
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

from scipy import stats
import matplotlib.pyplot as plt
import cv2.aruco as aruco



kinova_servicedef="""
#Service to provide simple interface to Baxter
service KinovaUI_interface

option version 0.4

struct BaxterImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end struct

struct CameraIntrinsics
    field double[] K
    field double[] D
end struct

struct ImageHeader
    field int32 width
    field int32 height
    field int32 step
end struct

struct ARtagInfo
    field double[] tmats
    field int32[] ids
end struct

object KinovaUI

    # property uint8 camera_open

    # camera control functions
    function double getInput()
    # function double[] getPos()
    # function double getDepth(double[] bbox)
    # function void testFunction()
    # function void ARtag_Detection()

    
end object

"""

data = None

class KinovaUI(object):
    """docstring for Kinova"""
    def __init__(self):
        self.joystick_address = "/joy"


        self.buttons = [0] * 8
        self.axes = [0] * 8
        # print self.joy_sub.data
        # self._ee_pos = [0]*3
        # self._ee_ori = [0]*4

        # # Initialize ARtag detection
        # self._aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        # self._arucoParams = aruco.DetectorParameters_create()
        # self._markerSize = 0.061

        # self._t_effector = threading.Thread(target=self.endeffector_worker)
        # self._t_effector.daemon = True
        # self._t_effector.start()
        # image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, process) 
        
        self.last_time = time.time()

        self._t_effector = threading.Thread(target=self.joystick_worker)
        self._t_effector.daemon = True
        self._t_effector.start()



    def close(self):
        # self._running = False
        # self._t_joints.join()
        self._t_effector.join()
        # self._t_command.join()

    def getInput(self):
        # resource = 1
        # print time.time()
        if time.time() - self.last_time > 0.15:
            axes = np.array(self.axes)
            index = np.argmax(np.abs(axes))
            value = axes[index]
            if index == 0 and value < -0.5 :
                self.last_time = time.time()
                return 2
            elif index == 0 and value > 0.5 :
                self.last_time = time.time()
                return -2
            elif index == 1:
                self.last_time = time.time()
                if value < -0.5:
                    return 1
                elif value > 0.5:
                    return -1
            else:
                return 0
        return 0


    def get_joystick(self, data):
        self.buttons = data.buttons
        self.axes = data.axes
        # rospy.sleep(1)


    def joystick_worker(self):
        self.joy_sub = rospy.Subscriber(self.joystick_address, Joy, self.get_joystick)


    def getPos(self):
        return self._ee_pos

    def getOri(self):
        while self._ee_ori == [0,0,0,0]:
            pass
        return self._ee_ori

    def process_img(self, data):
        depth_data, "32FC1"

    def getDepthImg(self):
        while self.depth_image is None:
            pass
        return self.depth_image

    def getImg(self):
        while self.image is None:
            pass
        return self.image

    def getDepth(self, bbox):
        bbox[2] *= 0.75
        bbox[3] *= 0.75
        bbox = [int(i) for i in bbox]
        print bbox
        print self.depth_image.shape

        # the depth img and color img can not be aligned well. So add the offset.
        '''
        bbox[0]: width initial point
        bbox[1]: height initial point
        '''
        sub_depth = self.depth_image[bbox[1]:bbox[3]+bbox[3], bbox[0]:bbox[2]+bbox[0]]
        sub_img = self.image[bbox[1]:bbox[3]+bbox[1], bbox[0]:bbox[2]+bbox[0]]

        # check the area location in depth img respect the coordinates in color img
        # cv2.rectangle(self.depth_image,(bbox[0],bbox[1]),(bbox[0]+bbox[2],bbox[1]+bbox[3]),(0,255,0),5)

        # for debug
        cv2.imwrite("sub_depth_img1.png", sub_depth)
        cv2.imwrite("depth_img.png", self.depth_image)
        # cv2.imshow("test", sub_depth)
        # cv2.imshow("test2", self.depth_image)
        # cv2.imshow("test3", sub_img)
        # cv2.imshow("test4", self.image)

        # cv2.waitKey(1000)
        # cv2.destroyAllWindows()
        print sub_depth.shape
        sub_depth = np.reshape(sub_depth, sub_depth.shape[0] * sub_depth.shape[1])
        print sub_depth.shape
        sub_depth = sub_depth[np.where( (sub_depth>100)&(sub_depth<400) )]
        result = np.median(sub_depth)
        print sub_depth
        print result
        return result

    # function that AR tag detection uses
    def getObjectPose(self, corners):
        with self._lock:
            camMatrix = numpy.reshape(self._camera_intrinsics.K, (3, 3))
            
            distCoeff = numpy.zeros((1, 5), dtype=numpy.float64)
            distCoeff[0][0] = self._camera_intrinsics.D[0]
            distCoeff[0][1] = self._camera_intrinsics.D[1]
            distCoeff[0][2] = self._camera_intrinsics.D[2]
            distCoeff[0][3] = self._camera_intrinsics.D[3]
            distCoeff[0][4] = self._camera_intrinsics.D[4]

            # print "cameramatrix: ", camMatrix
            # print "distortion coefficient: ", distCoeff

            # AR Tag Dimensions in object frame
            objPoints = numpy.zeros((4, 3), dtype=numpy.float64)
            # (-1, +1, 0)
            objPoints[0,0] = -1*self._markerSize/2.0 # -1
            objPoints[0,1] = 1*self._markerSize/2.0 # +1
            objPoints[0,2] = 0.0
            # (+1, +1, 0)
            objPoints[1,0] = self._markerSize/2.0 # +1
            objPoints[1,1] = self._markerSize/2.0 # +1
            objPoints[1,2] = 0.0
            # (+1, -1, 0)
            objPoints[2,0] = self._markerSize/2.0 # +1
            objPoints[2,1] = -1*self._markerSize/2.0 # -1
            objPoints[2,2] = 0.0
            # (-1, -1, 0)
            objPoints[3,0] = -1*self._markerSize/2.0 # -1
            objPoints[3,1] = -1*self._markerSize/2.0 # -1
            objPoints[3,2] = 0.0

            # Get each corner of the tags
            imgPoints = numpy.zeros((4, 2), dtype=numpy.float64)
            for i in range(4):
                imgPoints[i, :] = corners[0, i, :]

            # SolvePnP
            retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, camMatrix, distCoeff)
            Rca, b = cv2.Rodrigues(rvec)
            Pca = tvec

            # print "pca, rca: ", Pca, Rca

            return [Pca, Rca]

    def ARtag_Detection(self):
        # if not self.camera_open:
        #     self.openCamera()
        print "Detecting AR tags..."
        print self.image.shape
        currentImage = self.image[:,:,:]
         
        # imageData = numpy.reshape(imageData, (800, 1280, 4))
        gray = cv2.cvtColor(currentImage, cv2.COLOR_BGRA2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._arucoParams) 
        print ids
        if ids is not None:
            Tmat = []
            IDS = []
            detectioninfo = RR.RobotRaconteurNode.s.NewStructure("KinovaCamera_interface.ARtagInfo")
            for anid in ids:
                IDS.append(anid[0])
            for corner in corners:
                pc, Rc = self.getObjectPose(corner) 
                # here switch the position vector in the matrix
                Tmat.extend([   Rc[0][0],   Rc[1][0],   Rc[2][0],   0.0,
                                Rc[0][1],   Rc[1][1],   Rc[2][1],   0.0,
                                Rc[0][2],   Rc[1][2],   Rc[2][2],   0.0,
                                pc[1],      pc[0],      pc[2],      1.0])

            detectioninfo.tmats = Tmat
            detectioninfo.ids = IDS
            return detectioninfo

    def testFunction(self):
        depth_img = np.reshape(self.depth_image, self.depth_image.shape[0] * self.depth_image.shape[1])
        depth_img = depth_img[np.where((depth_img>0) & (depth_img<500) )]
        hist, bin_edges = np.histogram(depth_img, density=True)
        _ = plt.hist(depth_img, bins='auto')  # arguments are passed to np.histogram
        plt.title("Histogram with 'auto' bins")
        # Text(0.5, 1.0, "Histogram with 'auto' bins")
        plt.show()

    def process(self, data):
        pose = data.pose
        self._ee_pos[0] = pose.position.x
        self._ee_pos[0] = pose.position.x
        self._ee_pos[1] = pose.position.y
        self._ee_pos[2] = pose.position.z
        self._ee_ori[0] = pose.orientation.w
        self._ee_ori[1] = pose.orientation.x
        self._ee_ori[2] = pose.orientation.y
        self._ee_ori[3] = pose.orientation.z

    def readEndEffectorPoses(self):
        l_pose = self._left.endpoint_pose()
        if l_pose:
            self._ee_pos[0] = l_pose['position'].x
            self._ee_pos[1] = l_pose['position'].y
            self._ee_pos[2] = l_pose['position'].z
            self._ee_ori[0] = l_pose['orientation'].w
            self._ee_ori[1] = l_pose['orientation'].x
            self._ee_ori[2] = l_pose['orientation'].y
            self._ee_ori[3] = l_pose['orientation'].z
        r_pose = self._right.endpoint_pose()


    def callback(self, rgb_data, depth_data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError, e:
            print e

    # worker function to request and update end effector data for baxter
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        image_sub = rospy.Subscriber(self.tag_address, geometry_msgs.msg.PoseStamped, self.process) 
        self.ts.registerCallback(self.callback)
        # rospy.spin()


def main(argv):
    # parse command line arguments
    rospy.init_node('UI', anonymous = True)
    parser = argparse.ArgumentParser(description='Initialize kinova user-interface module.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on (will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the RobotRaconteur Node name
    RR.RobotRaconteurNode.s.NodeName="KinovaUIServer"

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
        RR.IPNodeDiscoveryFlags_LINK_LOCAL | RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(kinova_servicedef)
    
    #Initialize object
    kinova_obj = KinovaUI()
    
    RR.RobotRaconteurNode.s.RegisterService("UI", 
                "KinovaUI_interface.KinovaUI", kinova_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/KinovaUIServer" 
    raw_input("press enter to quit...\r\n")

    kinova_obj.close()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
    # export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH