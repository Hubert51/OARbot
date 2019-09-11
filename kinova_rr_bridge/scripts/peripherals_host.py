#!/usr/bin/env python
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
import tf2_ros
import tf_conversions

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

# from scipy import stats
# import matplotlib.pyplot as plt
import cv2.aruco as aruco



kinova_servicedef="""
#Service to provide simple interface to Baxter
service KinovaPeripheral_interface

option version 0.4
struct framePose
    field double[] position
    field double[] quaternion
end struct


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

object KinovaPeripherals
    function framePose lookUptransforms()

    # property uint8 camera_open

    # camera control functions
    # function double getInput()
    # function double[] getPos()
    # function double getDepth(double[] bbox)
    # function void testFunction()
    # function void ARtag_Detection()

    
end object

"""

data = None

class KinovaPeripherals(object):
    def __init__(self):
        print "Initializing Node"
        self._listener = tf.TransformListener()
        self._tfBuffer = tf2_ros.Buffer()
        self._listener2 = tf2_ros.TransformListener(self._tfBuffer)
        self.relativePose = RR.RobotRaconteurNode.s.NewStructure( 
                          'KinovaPeripheral_interface.framePose' )
        self._running = True
        self.last_time = time.time()

        self._t_effector = threading.Thread(target=self.tf_worker)
        self._t_effector.daemon = True
        self._t_effector.start()



    def close(self):
        self._running = False
        # self._t_joints.join()
        self._t_effector.join()
        # self._t_command.join()

    def tf_worker(self):
        rate = rospy.Rate(100.0)
        while self._running:
            try:
                T_base2goal = self._tfBuffer.lookup_transform('j2n6s300_link_base', 'goal_frame', rospy.Time())
                T_base2end = self._tfBuffer.lookup_transform('j2n6s300_link_base', 'j2n6s300_end_effector', rospy.Time())
                T_end2goal = self._tfBuffer.lookup_transform('j2n6s300_end_effector', 'goal_frame', rospy.Time())
                T_base2tag = self._tfBuffer.lookup_transform('j2n6s300_link_base', 'marker_frame', rospy.Time())

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                # publishControl(0, 0, 0, 0, 0, 0)
                continue
            print rospy.Time(), 'get tf'
            # Find Position Vector End-Effector(e) to Goal(g) wrt Base Link Frame(b).
            x_b = T_base2tag.transform.translation.x 
            y_b = T_base2tag.transform.translation.y 
            z_b = T_base2tag.transform.translation.z 
            
            P_e2g_b = [x_b, y_b, z_b]

            # Find Quaternion(Q) and XYZ Euler(E) Rotation between End-effector(e) to Goal(g)
            # Quaternion
            Q_eg = [T_base2goal.transform.rotation.x, T_base2goal.transform.rotation.y, T_base2goal.transform.rotation.z, T_base2goal.transform.rotation.w]
            
            self.relativePose.position = P_e2g_b
            self.relativePose.quaternion = Q_eg
            rate.sleep()



    # def lookUptransforms(self, target_frame, source_frame):
    def lookUptransforms(self):
        # position, quaternion = self._listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        # relativePose = RR.RobotRaconteurNode.s.NewStructure( 
        #                   'KinovaPeripheral_interface.framePose' )
        # relativePose.position = list(position)
        # relativePose.quaternion = list(quaternion)
        return self.relativePose


def main(argv):
    # parse command line arguments
    rospy.init_node('kinova_peripherals', anonymous = True)
    parser = argparse.ArgumentParser(description='Initialize kinova peripherals module.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on (will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the RobotRaconteur Node name
    RR.RobotRaconteurNode.s.NodeName="KinovaPeripheralsServer"

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
    kinova_obj = KinovaPeripherals()
    
    RR.RobotRaconteurNode.s.RegisterService("peripherals", 
                "KinovaPeripheral_interface.KinovaPeripherals", kinova_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/KinovaPeripheralsServer" 
    raw_input("press enter to quit...\r\n")

    kinova_obj.close()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
    # export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH
