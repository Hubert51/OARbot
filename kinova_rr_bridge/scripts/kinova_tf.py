#!/usr/bin/env python
import rospy
import numpy as np
import cv2

import math
from math import pi
import tf
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_from_matrix, inverse_matrix, euler_from_quaternion, euler_from_matrix

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import sys
import thread
import threading

# camera
from camera_host import KinovaCamera

data = None
prefix = "j2n6s300_"
# right joint position
position = [4.678701590491841, 3.8050904770432625, 0.9413227831121705, -2.2098248929946633, 1.6683761470320537, 8.621530226481784, 0.0012319970456220702, 0.0024639940912441404, 0.0]


class Kinova(object):
    """docstring for Kinova"""
    def __init__(self):
        self.address = "/j2n6s300_driver/out/tool_pose"

        self._ee_pos = [0]*3
        self._ee_ori = [0]*4
        self._pose = None
        self._valid = False

        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        # image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, process) 

    def getPos(self):
        while self._valid == False:
            rospy.sleep(0.1)
            pass
        return self._ee_pos

    def getOri(self):
        while self._valid == False:
            rospy.sleep(0.1)
            pass
        return self._ee_ori

    def process(self, data):
        pose = data.pose
        self._ee_pos[0] = pose.position.x
        self._ee_pos[0] = pose.position.x
        self._ee_pos[1] = pose.position.y
        self._ee_pos[2] = pose.position.z
        self._ee_ori[0] = pose.orientation.x
        self._ee_ori[1] = pose.orientation.y
        self._ee_ori[2] = pose.orientation.z
        self._ee_ori[3] = pose.orientation.w

        self._valid = True

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


    def cartesian_pose_client(self, position, orientation):
        """Send a cartesian goal to the action server."""
        action_address = '/' + prefix + 'driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None


    # worker function to request and update end effector data for baxter
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        image_sub = rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, self.process) 

        # while self._running:
        #     t1 = time.time()
        #     self.readEndEffectorPoses()
        #     self.readEndEffectorTwists()
        #     self.readEndEffectorWrenches()
        #     while (time.time() - t1 < 0.01):
        #         # idle
        #         time.sleep(0.001)



def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_

def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_

def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def rotm(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

if __name__ == '__main__':
    camera = KinovaCamera()


    hard_code = [0.51597311, 0.53908436, 0.45265119, 0.48812571]
    random_qs = [0.594, 0.219, 0.347, 0.692]
    qs2 = [-0.228, 0.605, -0.679, 0.348]
    no_transfrom = [-0.64293832 , 0.30220321, -0.34661193, 0.61250609]
    y_trans = [-0.32846894,  0.6292586,   0.64589147, -0.28100886]
    temp = [-0.34455515,  0.61269349,  0.6253454,  -0.33886807]
    # subScribe()
    rospy.init_node('kinova_cameras', anonymous = True)
    kinova = Kinova()

    rotm = np.dot(quaternion_matrix(qs2), inverse_matrix(quaternion_matrix(random_qs)))
    print rotm
    new_qs = np.dot(rotm, quaternion_matrix(kinova.getOri()))
    # kinova.cartesian_pose_client(kinova.getPos(), camera.getOri())

    print quaternion_matrix(hard_code)
    while 1:
        depth = camera.getDepthImg()
        img = camera.getImg()
        cv2.imshow('Body Recognition', depth[164:300, 323:443])
        cv2.imshow('Body Recognition2', img[323:443, 164:300, :])
        print depth[164:330, 323:443]
        cv2.waitKey(0)
        print 
        break
    sys.exit(1)

    # camera = KinovaCamera()
    # kinova.cartesian_pose_client(kinova.getPos(), hard_code)
    
    # test for rotate end effector
    kinova.cartesian_pose_client(kinova.getPos(), random_qs)
    # end2tag = quaternion_matrix(camera.getOri())
    base2arm = quaternion_matrix(kinova.getOri())
    my_rotm = np.dot(inverse_matrix(base2arm), rotation_matrix(1*pi, (1, 0, 0)) )
    # result= np.dot(my_rotm, end2tag)# axang2tform([0 1 0 -0.5*pi]);
    # result = np.dot(rotation_matrix(0.5*pi, (1, 0, 0)), result)
    # result = np.dot(rotation_matrix(0.5*pi, (0, 0, 1)), result)
    qs = tf.transformations.quaternion_from_matrix(my_rotm)
    # kinova.cartesian_pose_client(kinova.getPos(), qs)


    rospy.sleep(1)
    # print camera.getPos()
    sys.exit(1)

    # position
    my_rotm =  np.dot(rotm([1, 0, 0], 0.5*pi), rotm([0, 1, 0], -0.5*pi) )  # axang2tform([0 1 0 -0.5*pi]);
    my_rotm =  np.dot(rotm([1, 0, 0], 0.5*pi), rotm([0, 1, 0], -0.5*pi) )  # axang2tform([0 1 0 -0.5*pi]);

    v = [0, 0.3, 0.]
    # axis = [4, 4, 1]
    # theta = 1.2 
    # v = np.dot(my_rotm, v)
    # print() 
    # my_rotm = 
    # v = np.dot(my_rotm, v)
    # print(v)
    # [ 2.74911638  4.77180932  1.91629719]

    # print camera.getOri()
    # orientation
    # end2tag = tf.transformations.quaternion_matrix([0.52, -0.51, 0.48, -0.47])
    # print euler_from_quaternion(camera.getOri())
    end2tag = quaternion_matrix(camera.getOri())
    base2arm = quaternion_matrix(kinova.getOri())
    my_rotm = np.dot(rotation_matrix(0.5*pi, (0,0,1)), rotation_matrix(-0.5*pi, (1, 0, 0)))
    my_rotm = rotation_matrix(0*pi, (0, 0, 1))
    my_rotm = np.dot(inverse_matrix(base2arm), rotation_matrix(1*pi, (1, 0, 0)) )
    print my_rotm
    # print my_rotm
    print np.dot(my_rotm[0:3,0:3], v)

    # my_rotm[0:3, 0:3] = np.dot(rotation_matrix(0.5*pi, (1, 0, 0))[0:3,0:3], my_rotm[0:3, 0:3])

    # my_rotm = rotation_matrix(0.5*pi, (0, 0, 1))
    result = np.identity(4)
    base2end = quaternion_matrix(kinova.getOri())
    result= np.dot(my_rotm, end2tag)# axang2tform([0 1 0 -0.5*pi]);
    # result = np.dot(rotation_matrix(0.5*pi, (1, 0, 0)), result)
    # result = np.dot(rotation_matrix(0.5*pi, (0, 0, 1)), result)

    qs = tf.transformations.quaternion_from_matrix(result)
    print qs
    euler = euler_from_matrix(result)
    orientation_ = euler
    # orientation_ = [euler[1], euler[2], euler[0]    print euler
    
    orientation_rad_list =  Quaternion2EulerXYZ(kinova.getOri())
    orientation_rad = [orientation_rad_list[i] - orientation_[i] for i in range(0,3)]
    orientation_deg = list(map(math.degrees, orientation_rad))
    orientation_deg[0] += 90
    orientation_q = EulerXYZ2Quaternion(orientation_rad)
    
    print orientation_q

    qs_matrix = quaternion_matrix(qs)
    print qs_matrix
    # kinova.cartesian_pose_client(kinova.getPos(), [qs[1], qs[2], qs[3], qs[0]])
    ori = kinova.getOri()
    ori_matrix = quaternion_matrix(ori)
    qs_matrix[0:3, 0:3] = np.dot(qs_matrix[0:3,0:3], ori_matrix[0:3,0:3])
    # result = quaternion_from_matrix(qs_matrix)


    # euler_angle = euler_from_quaternion(result)
    # print euler_angle
    # print result
    print ori
    kinova.cartesian_pose_client(kinova.getPos(), qs)

    # euler_angle = euler_from_quaternion([qs[1], qs[2], qs[3], qs[0]])
    # print euler_angle
