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
import geometry_msgs
import sensor_msgs
from sensor_msgs.msg import JointState
import tf2_ros
import tf
from math import pi


# print os.getcwd()

class CollisionCheck(object):
    def __init__(self):
        # initial ros node
        rospy.init_node('collision_check', anonymous = True)

        # get joint state data
        self.jointStateAddress = "/j2n6s300_driver/out/joint_state"
        self.joint_pos = np.array([0.0] * 6)
        self.kinova_joint_pos = np.array([0.0] * 6)
        rospy.Subscriber(self.jointStateAddress, JointState, self.jointFeedback)

        # get fridge location relative to the kinova base link
        # try:
        self._listener = tf.TransformListener()
        self._tfBuffer = tf2_ros.Buffer()
        self._listener2 = tf2_ros.TransformListener(self._tfBuffer)
        startTime = time.time()
        while True:
            if time.time() - startTime > 10:
                fridge_pos = [0.1568307262, 0.626700624618, 0.308028843394]
                print "Can not find tfBuffer. Use pre-defined value"
                break
            try:
                T_base2tag = self._tfBuffer.lookup_transform('j2n6s300_link_base', 'marker_frame', rospy.Time())
                fridge_pos = np.array([ T_base2tag.transform.translation.x,
                                        T_base2tag.transform.translation.y,
                                        T_base2tag.transform.translation.z
                                        ])
                print "Get correct tf from base to marker"
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
                # print err
                pass

        # except:
        #     tag_pos = [0.1568307262, 0.626700624618, 0.308028843394]


        rospy.sleep(1)

        self.physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)

        kinovaStartPos = [0,0,0.675]
        kinovaStartOri = p.getQuaternionFromEuler([0,0,0])

        # two objects
        self.planeId = p.loadURDF("plane.urdf") # environment the file is in bullet default folder
        self.kinova = p.loadURDF("data/model2.urdf", kinovaStartPos, kinovaStartOri, useFixedBase=True)
        self.box = p.loadURDF("data/table.urdf", [0,1.4,0.38], kinovaStartOri, useFixedBase=True)

        # self.box = p.loadURDF("data/cube.urdf", [0,1.3,0],cubeStartOrientation)
        # self.box = p.loadSDF("data/ketchen.model")
        # self.box = p.loadURDF("data/kitchen_object.urdf")
        # self.fridge = p.loadURDF("kitchen/fridge.urdf", [0,1.7,1],cubeStartOrientation)
        # self.joint_pos = np.array([0., pi/2, pi/2, pi/2, pi/2, 0])

        self.fridge = p.loadURDF("kitchen/fridge.urdf", [fridge_pos[0],fridge_pos[1]+0.075,fridge_pos[2]-0.18+0.675],kinovaStartOri, useFixedBase=True )

        self.kinovaEndEffector = 7
        self.jacobian = None
        self.closestPts = None
        p.setRealTimeSimulation(1)

        # self._t_effector = threading.Thread(target=self.simulation)
        # self._t_effector.daemon = True
        # self._t_effector.start()

    def getKinovaJointState(self, unit="radius"):
        if unit[0] == "r":
            return self.kinova_joint_pos
        elif unit[0] == "d":
            return self.kinova_joint_pos * 180 / pi
        else:
            raise Exception("Unknown Unit!")

    def jointFeedback(self, data):
        pos = data.position
        for i in range(6):
            value = data.position[i]
            self.kinova_joint_pos[i] = value
            while value > 2*pi:
                value -= 2*pi
            while value < -2*pi:
                value += 2*pi
            self.joint_pos[i] = value


    def simulation2(self):
        # if we do not run real arm. This is the pre-defined value of joint position
        if np.nonzero(self.joint_pos)[0].size == 0:
            self.joint_pos = np.array([2.9654557064658054, 2.9599693283041333, 0.5794236254734035, 3.9531707557671565, 1.8754357233632242, 1.953975539702969])
        p.setTimeStep(0.1)
        for i in range(len(self.joint_pos)):
            p.setJointMotorControl2(self.kinova,
                                    i+1,
                                    p.POSITION_CONTROL,
                                    targetPosition=self.joint_pos[i]
                                    )
        time.sleep(7)

    def velocity_control(self):
        p.setJointMotorControl2(self.kinova,
                                6,
                                p.VELOCITY_CONTROL,
                                targetVelocity=0.1
                                )
        for i in range(1000):
            p.stepSimulation()
            p.resetBasePositionAndOrientation(self.kinova, (0,0,0), (0.0, 0.0, 0.0, 1.0))
        pass

    def collision_report(self):
        closestPts = p.getClosestPoints(self.kinova, self.fridge, 0.5)
        sorted_pts = sorted(closestPts, key=lambda pt: pt[8])
        numerOfFridge = p.getNumJoints(self.fridge)
        # define a set of closest points.
        resultClosestPts = dict()
        for i in range(-1,numerOfFridge):
            resultClosestPts[i] = None
            # print p.getJointInfo(self.fridge, i)[1]
        print "length is {}".format(len(sorted_pts))
        j = len(resultClosestPts)
        i = 0
        while j > 0 or i < 20:
            # print sorted_pts[i][3], sorted_pts[i][4]
            # print np.array(sorted_pts[i][5]) - np.array([0,0,0.675])
            # print np.array(sorted_pts[i][6]) - np.array([0,0,0.675])
            # print np.array(sorted_pts[i][5]) - np.array(sorted_pts[i][6]), sorted_pts[i][8]
            # print
            if resultClosestPts[sorted_pts[i][4]] is None:
                print np.array(sorted_pts[i][5]) - np.array([0,0,0.675])
                print np.array(sorted_pts[i][6]) - np.array([0,0,0.675])
                print np.array(sorted_pts[i][5]) - np.array(sorted_pts[i][6]), sorted_pts[i][8]
                print
                resultClosestPts[sorted_pts[i][4]] = (np.array(sorted_pts[i][5]), np.array(sorted_pts[i][6]))
                j -= 1
            i += 1
        # sorted_pts[0][5] is first object in the getClosestPoints method which is kinova
        # sorted_pts[0][6] is first object in the getClosestPoints method which is box
        return resultClosestPts

    def calculateInverseKinematics(self, tar_pos, tar_ori=None):
        for i in range(14):
            data = p.getJointInfo(self.kinova, i)
        q = p.calculateInverseKinematics(self.kinova,
                                         endEffectorLinkIndex=7,
                                         targetPosition=tar_pos,
                                         targetOrientation=tar_ori,
                                         maxNumIterations=100)
        # print q
        return np.array(q[0:6])

    def update_joint(self, joint_pos, type="relative"):
        if type == "relative":
            joint_pos += self.joint_pos
        for i in range(len(self.joint_pos)):
            p.setJointMotorControl2(self.kinova,
                                    i+1,
                                    p.POSITION_CONTROL,
                                    targetPosition=joint_pos[i]
                                    )
        (base_pos, base_ori) = p.getBasePositionAndOrientation(self.kinova)
        # for i in range(1000):
        #     p.stepSimulation()

    def calculateJacobian(self, q):
        pos, vel, torq = self.getJointStates(self.kinova)
        mpos, mvel, mtorq = self.getMotorJointStates(self.kinova)
        # get the end effector link state
        result = p.getLinkState(self.kinova,
                                self.kinovaEndEffector,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        zero_vec = [0.0] * len(mpos)
        jac_t, jac_r = p.calculateJacobian(self.kinova, self.kinovaEndEffector, com_trn, mpos, zero_vec, zero_vec)
        self.jacobian = jac_t
        # calculate the linear velocity of the endeffector
        print(self.multiplyJacobian(self.kinova, jac_t, vel))
        return (jac_t, jac_r)

    def setJointPosition(self, position, kp=1.0, kv=0.01):
        num_joints = 6 # p.getNumJoints(self.kinova)
        zero_vec = [0.0] * num_joints
        p.setTimeStep(0.1)
        if len(position) == 6:
            p.setJointMotorControlArray(self.kinova,
                                        range(1,num_joints+1),
                                        p.POSITION_CONTROL,
                                        targetPositions=position,
                                        targetVelocities=zero_vec,
                                        positionGains=[kp] * num_joints,
                                        velocityGains=[kv] * num_joints)
            # for i in range(100):
            #     p.stepSimulation(0.001)

        else:
            print("Not setting torque. "
                  "Expected torque vector of "
                  "length {}, got {}".format(num_joints, len(torque)))

    def getMotorJointStates(self, robot):
        joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
        joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def getJointStates(self, robot):
        joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def multiplyJacobian(self, robot, jacobian, vector):
        """
        This function is for in-class call
        :param robot:
        :param jacobian:
        :param vector:
        :return:
        """
        result = [0.0, 0.0, 0.0]
        i = 0
        # for c in range(len(vector)):
        for c in range(1,7):
            if p.getJointInfo(robot, c)[3] > -1:
                for r in range(3):
                    result[r] += jacobian[r][i] * vector[c]
                i += 1
        return result


    def multiplyJacobian2(self, vector):
        """
        This function is for out class call
        :param vector:
        :return:
        """
        result = [0.0, 0.0, 0.0]
        i = 0
        # for c in range(len(vector)):
        for c in range(len(vector)):
            if p.getJointInfo(self.kinova, c)[3] > -1:
                for r in range(3):
                    result[r] += self.jacobian[r][i] * vector[c]
                i += 1
        return result



if __name__ == '__main__':
    Checker = CollisionCheck()
    Checker.simulation2()
    while True:
        Checker.velocity_control()
        raw_input("pause")
    # Checker.collision_report()
    raw_input("pause")
    sys.exit(1)

    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0, -10)
    planeId = p.loadURDF("plane.urdf")
    # planeId = p.loadURDF("model2.urdf")
    # planeId = p.loadURDF("/home/ruijie/kinova_ws/src/object_detection/scripts/plane.urdf")


    cubeStartPos = [0,0,1]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    # boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
    kinova = p.loadURDF("model2.urdf",cubeStartPos, cubeStartOrientation)
    box = p.loadURDF("husky/husky.urdf", [0,-1,2],cubeStartOrientation)

    # for i in range (p.getNumJoints(boxId)):
    #     jointInfo=p.getJointInfo(boxId,i)
    #     print("joint",jointInfo[0],"name=",jointInfo[1].decode('ascii'))

    joint_pos = np.array([0., pi/2, pi/2, pi/2, pi/2, 0])
    # joint_pos = np.array([0., 0, 0, 0, 0, 0])


    # joint_pos = p.calculateInverseKinematics(boxId, 6, [-0.00213531428017, 0.0643087550998, 1.18114089966], [0,0,0,1])
    print joint_pos

    p.setRealTimeSimulation(1)
    # while (1):
    for i in range(len(joint_pos)):
        p.setJointMotorControl2(kinova,
                                i,
                                p.POSITION_CONTROL,
                                targetPosition=joint_pos[i]
                                )
    (base_pos, base_ori) = p.getBasePositionAndOrientation(kinova)
    p.resetBasePositionAndOrientation(kinova, base_pos, (0.0, 0.0, 0.0, 1.0))
    point = p.getClosestPoints(kinova, box, 0.5)
    # print point[0][8]
    print len(point)
    time.sleep(0.001)

    raw_input("pause")
    i = 0
    p.setRealTimeSimulation(1)
    while True:
        if i % 10 == 0:
            # raw_input()
            pass
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        print(cubePos,cubeOrn)
        time.sleep(1)

    p.disconnect()
#
# import pybullet as p
# import time
# import pybullet_data
# if __name__ == '__main__':
#
#     physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
#     p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
#     p.setGravity(0,0,-10)
#     planeId = p.loadURDF("plane.urdf")
#     cubeStartPos = [0,0,1]
#     cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#     boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
#     for i in range (10000):
#         p.stepSimulation()
#     time.sleep(1./240.)
#     cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#     print(cubePos,cubeOrn)
#     p.disconnect()

"""
Useful code
for i in range(14):
    data = p.getJointInfo(self.kinova, i)

"""

# def simulation(self):
#     while True:
#         p.setRealTimeSimulation(1)
#         # while (1):
#         for i in range(len(self.joint_pos)):
#             p.setJointMotorControl2(self.kinova,
#                                     i,
#                                     p.POSITION_CONTROL,
#                                     targetPosition=self.joint_pos[i]
#                                     )
#         (base_pos, base_ori) = p.getBasePositionAndOrientation(self.kinova)
#         p.resetBasePositionAndOrientation(self.kinova, base_pos, p.getQuaternionFromEuler([0,0,0]))
#         self.closestPts = p.getClosestPoints(self.kinova, self.box, 0.5)
#         time.sleep(0.001)
