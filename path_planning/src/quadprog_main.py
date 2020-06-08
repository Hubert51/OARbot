#!/usr/bin/env python2

import numpy as np
import numpy
from numpy.linalg import inv
from scipy.linalg import logm, norm, sqrtm
from ControlParams import *
# from OpenRAVE_rr_server import *
#import rpi_abb_irc5
import time
import timeit
from pyquaternion import Quaternion
import quadprog
import math, sys
from math import *
from collision_check import CollisionCheck

import rospy

# for controling the robot arm
import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> numpy.allclose(numpy.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> numpy.allclose(numpy.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # noqa: validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = numpy.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print(axes, "failed")

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # noqa: validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def robotParams():
    I3 = np.eye(3)
    ex = I3[:,0]
    ey = I3[:,1]
    ez = I3[:,2]

    h1 = -ez
    h2 = -ey
    h3 = ey
    h4 = -ez
    h5 = -np.array([0, math.sqrt(3)/2, 0.5])
    # h5 = ez
    # h6 = np.array([0, 0.5, math.sqrt(3)/2])
    # h6 = np.array([0, math.sqrt(3)/2, 0.5])

    h6 = -ez
    constant = 30*pi/180
    ## maybe correct
    P = np.array([[0,0,0], [0, 0, 0.2755], [0, -0.0098, -0.410], [0,0,0.2073], [0, sin(constant)*0.0741, cos(constant)*0.0741], [0, sin(constant)*0.0741, cos(constant)*0.0741], [0, 0, 0.16]]).T

    # P = np.array([ [0, 0, 0.2755], [-0.0098, -0.0098, 0.410], [0,0,0.2073], [0, sin(constant)*0.0741, cos(constant)*0.0741], [0, sin(constant)*0.0741, cos(constant)*0.0741], [0, -0.16*cos(constant), 0.16*sin(constant)], [0,0,0]]).T
    # P = np.array([[0,0,0], [0, 0, 0.2755], [0, -0.0098, -0.410], [0,0,0.2073], [0, 0, -0.0741], [0, 0, -0.0741], [0, 0, 0.16]]).T

    q = np.zeros((6, 1))
    H = np.array([h1, h2, h3, h4, h5, h6]).T
    ttype = np.zeros((1, 6))
    """ """
    n = 6

    dq_bounds = np.array([[100,110], [90,90], [90,90], [170,190], [120,140], [190,235]]).T
    dq_bounds = dq_bounds*np.pi/180

    return ex,ey,ez,n,P,q,H,ttype,dq_bounds

def fwdkin(q,ttype,H,P,n):
    R=np.eye(3)
    # print R
    p=np.zeros((3,1))

    for i in range(n):
        h_i = H[0:3,i].reshape(3, 1)
        Ri = np.eye(3)

        if ttype[0][i] == 0:
            #rev
            pi = P[0:3,i].reshape(3, 1)
            p = p+np.dot(R, pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R, Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1:
            # pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R, pi)
        else:
            # default pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R, pi)
        # print "joint {} is \n{}\n".format(i, p)

    # End Effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))

    return R, p

# find closest rotation matrix
# A=A*inv(sqrt(A'*A))
def Closest_Rotation(R):
    a = np.dot(R.T, R)
    sqrtm(np.dot(R.T, R))
    inv(sqrtm(np.dot(R.T, R)))
    R_n = np.dot(R, inv(sqrtm(np.dot(R.T, R))))

    return R_n

# ROT Rotate along an axis h by q in radius
def rot(h, q):
    h=h/norm(h)
    if h.shape == (3,1):
        h = np.transpose(h)[0]

    # R = np.eye(3) + np.sin(q)*hat(h) + (1 - np.cos(q))*np.dot(hat(h), hat(h))
    R = np.eye(3) + np.sin(q[0])*hat(h) + (1 - np.cos(q[0]))*np.dot(hat(h), hat(h))
    # print np.eye(3) + sin(q[0])*hat(h)
    # print q[0]
    return R

def hat(h):
    h_hat = np.array([[0, -h[2], h[1]], [h[2], 0, -h[0]], [-h[1], h[0], 0]])

    return h_hat

def fwdkin_alljoints(q, ttype, H, P, n):
    R=np.eye(3)
    p=np.zeros((3,1))
    RR = np.zeros((3,3,n+1))
    pp = np.zeros((3,n+1))

    for i in range(n):
        h_i = H[0:3,i]

        if ttype[0][i] == 0:
            #rev
            pi = P[0:3,i].reshape(3, 1)
            p = p+np.dot(R,pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R,Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1:
            #pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R,pi)
        else:
            # default pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R,pi)

        pp[:,[i]] = p
        RR[:,:,i] = R

    # end effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))
    pp[:,[n]] = p
    RR[:,:,n] = R

    return pp, RR

def Joint2Collision(Closest_Pt,pp):
    link_dist = []

    for i in range(5):
        link = pp[:,i+1]-pp[:,i]
        link = link/norm(link)
        pp2c = Closest_Pt - pp[:,i]

        link_dist.append(norm(pp2c - abs(np.dot(pp2c, link))*link))

    J2C_Joint = link_dist.index(min(link_dist)) + 1
    if(J2C_Joint==1):
        J2C_Joint=2

    return J2C_Joint

def getJacobian(q,ttype,H,P,n):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)

    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))

    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))
    """ """

    return J

""" """
# return jacobian of the closest point on panel
def getJacobian3(q,ttype,H,P,n, Closest_Pt, J2C_Joint):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)
    """  """

    P_0_T = Closest_Pt

    J = np.zeros((6,num_joints))

    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))

    return J

# return jacobian of the closest point on robot
def getJacobian2(q,ttype,H,P,n,Closest_Pt,J2C_Joint):

    num_joints = len(q)

    P_0_i,R_0_i = fwdkin_alljoints(q,ttype,H,P,n)

    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))

    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i], H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))

    J[:,J2C_Joint:7] = 0
    link_c = P_0_i[:,J2C_Joint]-P_0_i[:,J2C_Joint-1]
    link_c = link_c/norm(link_c)

    P_0_tmp = P_0_i[:,J2C_Joint-1]+ abs(np.dot(Closest_Pt-P_0_i[:,J2C_Joint-1],link_c))*link_c

    return J,P_0_tmp

# convert a unit quaternion to angle/axis representation
def quat2axang(q):

    s = norm(q[0][1:4])
    if s >= 10*np.finfo(np.float32).eps:
        vector = q[0][1:4]/s
        theta = 2*np.arctan2(s,q[0][0])
    else:
        vector = np.array([0,0,1])
        theta = 0
    axang = np.hstack((vector,theta))

    return axang


def getqp_f_new(er, ep):
    f = -2*np.array([0,0,0,0,0,0,er,ep]).reshape(8, 1)

    return f

def getqp_H_new(J, vr, vp, er, ep):
    n = 6

    H1 = np.dot(np.hstack((J,np.zeros((n,2)))).T,np.hstack((J,np.zeros((n,2)))))

    tmp = np.vstack((np.hstack((np.hstack((np.zeros((3,n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,n)),np.zeros((3,1)))),vp))))
    H2 = np.dot(tmp.T,tmp)

    H3 = -2*np.dot(np.hstack((J,np.zeros((n,2)))).T, tmp)
    H3 = (H3+H3.T)/2;

    tmp2 = np.vstack((np.array([0,0,0,0,0,0,np.sqrt(er),0]),np.array([0,0,0,0,0,0,0,np.sqrt(ep)])))
    H4 = np.dot(tmp2.T, tmp2)

    H = 2*(H1+H2+H3+H4)

    return H

def getqp_H(dq, J, vr, vp, er, ep):
    n = len(dq)
    H1 = np.dot(np.hstack((J,np.zeros((6,2)))).T,np.hstack((J,np.zeros((6,2)))))

    tmp = np.vstack((np.hstack((np.hstack((np.zeros((3,n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,n)),np.zeros((3,1)))),vp))))
    H2 = np.dot(tmp.T,tmp)

    H3 = -2*np.dot(np.hstack((J,np.zeros((6,2)))).T, tmp)
    H3 = (H3+H3.T)/2;

    tmp2 = np.vstack((np.array([0,0,0,0,0,0,np.sqrt(er),0]),np.array([0,0,0,0,0,0,0,np.sqrt(ep)])))
    H4 = np.dot(tmp2.T, tmp2)

    H = 2*(H1+H2+H3+H4)

    return H

def getqp_f(dq, er, ep):
    f = -2*np.array([0,0,0,0,0,0,er,ep]).reshape(8, 1)

    return f

def inequality_bound(h,c,eta,epsilon,e):
    sigma = np.zeros((h.shape))
    h2 = h - eta
    sigma[np.array(h2 >= epsilon)] = -np.tan(c*np.pi/2)
    sigma[np.array(h2 >= 0) & np.array(h2 < epsilon)] = -np.tan(c*np.pi/2/epsilon*h2[np.array(h2 >= 0) & np.array(h2 < epsilon)])
    sigma[np.array(h >= 0) & np.array(h2 < 0)] = -e*h2[np.array(h >= 0) & np.array(h2 < 0)]/eta
    sigma[np.array(h < 0)] = e

    return sigma

# quaternion multiply
def quatmultiply(q1, q0):
    w0, x0, y0, z0 = q0[0][0], q0[0][1], q0[0][2], q0[0][3]
    w1, x1, y1, z1 = q1[0][0], q1[0][1], q1[0][2], q1[0][3]

    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64).reshape(1, 4)


def is_pos_def(x):
    return np.all(np.linalg.eigvals(x)>0)


def QP_abbirb6640(q,v,check_flag=False, collision_checker=None):
    # Init the joystick

    # Initialize Robot Parameters
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams()
    # joint limits
    lower_limit = np.transpose(np.array([-360*np.pi/180, -360*np.pi/180, -2*np.pi, -300*np.pi/180, -180*np.pi/180, -2*np.pi]))
    upper_limit = np.transpose(np.array([360*np.pi/180, 360*np.pi/180, 360*np.pi/180, 300*np.pi/180, 180*np.pi/180, 2*np.pi]))

    # Initialize Control Parameters
    # initial joint angles

    pos_v = np.zeros((3, 1))
    ang_v = np.array([1,0,0,0])
    dq = np.zeros((int(n),1))

    # inequality constraints
    # we use h to calculate the sigma, so h should also be a vector of (13*1)
    # h consists of 6 upper bound, 6 lower bound and 1 distance constraints
    h = np.zeros((17, 1))
    sigma = np.zeros((17, 1))
    dhdq = np.vstack((np.hstack((np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.hstack((-np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.zeros((5, 8))))

    # velocities
    w_t = np.zeros((3, 1))
    v_t = np.zeros((3, 1))

    # keyboard controls
    # define position and angle step
    inc_pos_v = 0.01 # m/s
    inc_ang_v = 0.5*np.pi/180 # rad/s

    # optimization params
    er = 0.05
    ep = 0.05
    epsilon = 0 # legacy param for newton iters

    # parameters for inequality constraints
    c = 0.5
    eta = 0.1
    epsilon_in = 0.15
    E = 0.005

    pp,RR = fwdkin_alljoints(q,ttype,H,P,n)
    orien_tmp = Quaternion(matrix=RR[:, :, -1])
    orien_tmp = np.array([orien_tmp[0], orien_tmp[1], orien_tmp[2], orien_tmp[3]]).reshape(1, 4)

    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pp[:, -1],orien_tmp,pos_v,ang_v.reshape(1, 4),w_t,v_t,epsilon,inc_pos_v,inc_ang_v,0,er,ep,0)


    J_eef = getJacobian(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])


    # desired rotational velocity
    vr = v[0:3]

    # desired linear velocity
    vp = v[3:6]

    Q = getqp_H_new(J_eef, vr.reshape(3, 1),  vp.reshape(3, 1), obj.params['opt']['er'], obj.params['opt']['ep'])

    # make sure Q is symmetric
    Q = 0.5*(Q + Q.T)

    f = getqp_f_new(obj.params['opt']['er'], obj.params['opt']['ep'])
    f = f.reshape((8, ))


    # bounds for qp
    if obj.params['opt']['upper_dq_bounds']:
        bound = obj.params['defi']['dq_bounds'][1, :]
    else:
        bound = obj.params['defi']['dq_bounds'][0, :]

    LB = np.vstack((-0.1*bound.reshape(6, 1),0,0))
    UB = np.vstack((0.1*bound.reshape(6, 1),1,1))

    # inequality constrains A and b
    h[0:6] = obj.params['controls']['q'] - lower_limit.reshape(6, 1)
    h[6:12] = upper_limit.reshape(6, 1) - obj.params['controls']['q']
    sigma[0:12] = inequality_bound(h[0:12], c, eta, epsilon_in, E)

    # collision constraint
    if check_flag:
        # (Closest_Pt, Closest_Pt_env) = collision_checker.collision_report()
        closest_Pts = collision_checker.collision_report()
        i = 12
        for key, value in closest_Pts.items():
            Closest_Pt, Closest_Pt_env = value
            dist = numpy.linalg.norm(Closest_Pt_env-Closest_Pt)

            ## TODO: check several points instead of one point
            dx = Closest_Pt_env[0] - Closest_Pt[0]
            dy = Closest_Pt_env[1] - Closest_Pt[1]
            dz = Closest_Pt_env[2] - Closest_Pt[2]
            # dz = 0
            """ """
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            # assert dist == dist1
            # derivative of dist w.r.t time
            der = np.array([dx/dist, dy/dist, dz/dist])
            h[i] = dist
            dhdq[i, 0:6] = np.dot(-der[None, :], J_eef[3:6,:])
            # TODO: here I changed the equation of the sigma
            eta = 0.03
            sigma[i] = inequality_bound(h[i], c, eta, epsilon_in, E)
            # sigma[i] =  h[i]
            i += 1
            # break

    A = np.vstack((dhdq,np.eye(8), -np.eye(8)))
    b = np.vstack((sigma, LB, -UB))

    # solve the quadprog problem
    try:
        dq_sln = quadprog.solve_qp(Q, -f, A.T, b.reshape((33, )))[0]
    except:
        return [0,0,0,0,0,0]

    if len(dq_sln) < obj.params['defi']['n']:
        obj.params['controls']['dq'] = np.zeros((6,1))
        V_scaled = 0
        print 'No Solution'
        dq_sln = np.zeros((int(n),1))
    else:
        obj.params['controls']['dq'] = dq_sln[0: int(obj.params['defi']['n'])]
        obj.params['controls']['dq'] = obj.params['controls']['dq'].reshape((6, 1))
        #print dq_sln
    #        V_scaled = dq_sln[-1]*V_desired
    #        vr_scaled = dq_sln[-2]*vr.reshape(3,1)

    #print np.dot(np.linalg.pinv(J_eef),v)

    # get the linear velocity and angular velocity
    V_linear = np.dot(J_eef[3:6,:], obj.params['controls']['dq'])
    V_rot = np.dot(J_eef[0:3,:], obj.params['controls']['dq'])

    return dq_sln[0:6]

def main():

    # OpenRAVE_obj = OpenRAVEObject()

    # Initialize Robot Parameters
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams()

    # Initialize Control Parameters
    # initial joint angles

    """ """
    #    Q_all = np.array([[-1.5832209587097168, 0.35797879099845886, -0.018985196948051453, 3.492117684800178e-05, 1.2321044206619263, -0.012819867581129074],
    #    [-1.0001745223999023, 0.3400745689868927, 0.27556654810905457, -0.01793256774544716, 0.8835334777832031, -0.06672105938196182],
    #    [-0.4340008497238159, 0.33434557914733887, 0.2755616009235382, -0.01793256774544716, 0.8835381269454956, -0.06672105938196182],
    #    [-0.08396485447883606, 0.33433452248573303, 0.2755637466907501, -0.017921535298228264, 0.8835418224334717, -0.06673289835453033],
    #    [0.027317576110363007, 0.7219697833061218, -0.3032461106777191, -0.025421472266316414, 1.0768276453018188, 0.044921014457941055],
    #    [0.5158949494361877, 1.2091666460037231, -0.9566897749900818, -0.04086354747414589, 1.269188404083252, 0.5332122445106506]])
    #    Q_all = np.array([[-1.4735171794891357, 0.43576669692993164, 0.025293484330177307, 0.012627056799829006, 1.0394766330718994, 0.0606585256755352],
    #    [-1.040715515613556, 0.43576669692993164, 0.025293484330177307, 0.012627056799829006, 1.0394766330718994, 0.0606585256755352],
    #    [-0.6079138517379761, 0.4357639253139496, 0.025260837748646736, 0.030262663960456848, 1.039481282234192, 0.060406409204006195],
    #    [-0.607966959476471, 0.5187827348709106, 0.12658417224884033, 0.03462858498096466, 0.8553994297981262, -0.6607409119606018],
    #    [-0.4619627594947815, 1.0956398248672485, -0.7398647665977478, 0.01724303886294365, 1.1420093774795532, -0.49965518712997437]])

    Q_all = np.array([[-1.5832209587097168, 0.35797879099845886, -0.018985196948051453, 3.492117684800178e-05, 1.2321044206619263, -0.012819867581129074],
                      [-1.040715515613556, 0.35797879099845886, -0.018985196948051453, 3.492117684800178e-05, 1.2321044206619263, -0.012819867581129074],
                      [-0.6079138517379761, 0.35797879099845886, -0.018985196948051453, 3.492117684800178e-05, 1.2321044206619263, -0.012819867581129074],
                      [-0.607966959476471, 0.5187827348709106, 0.12658417224884033, 0.03462858498096466, 0.8553994297981262, -0.6607409119606018],
                      [-0.4619627594947815, 1.0956398248672485, -0.7398647665977478, 0.01724303886294365, 1.1420093774795532, -0.49965518712997437]])
    test_q = np.array( [4.8046852, 2.92482, 1.002, 4.2031852, 1.4458, 1.3233])
    test_q = np.array([3.464029, 2.65480, 0.514936, 3.10589, 1.3006, 2.48233])
    test_q = np.array([0.0019249955138216472, 3.1432288997635376, 3.139667720753884, 3.1451626210310444, 3.1404027532147456, 0.004759989010934167])
    test_q = np.array([0., pi, pi, pi, pi, 0])

    # calibrate 1:
    # real x: -0.00213531428017 y: 0.0643087550998 z: 1.18114089966
    # predict x: -1.71608523e-16 y: 6.43000000e-02 z: 1.18114496e+00
    # test_q = np.array([0., pi, pi, 0, 0, 0])

    # calibrate 2:
    # real: x: 0.498562157154 y: 0.0647798776627 z: 0.679222285748
    # our predict: x: 0.49564496 y: 0.0643 z: 0.6855
    # test_q = np.array([0., pi, 1.5*pi, 0, 0, 0])

    # calibrate 3:
    # real x: 0.398912936449 y: -0.305943399668 z: 0.677314043045
    # our predict x: 0.39594088 y: -0.30500695 z: 0.6855
    # test_q = np.array([pi/4., pi, 1.5*pi, 0, 0, 0])

    # calibrate 4:
    # real: x: 0.349278271198 y: -0.256594508886 z: 0.431439548731
    # our predict: x: 0.34898628 y: -0.25805235 z: 0.43767752
    # test_q = np.array([pi/4., pi, pi*5/3, 0, 0, 0]) # 45, 180, 300, 0, 0, 0

    # calibrate 5:
    # real: x: 0.382904559374 y: -0.165705829859 z: 0.657457232475
    # predict: x: 0.38003075 y: -0.16491893 z:0.66580688
    test_q = np.array([pi/4., pi, pi*5/3, 0, pi/2, 0]) # 45, 180, 300, 0, 90, 0

    # calibrate 6:
    # real x: 0.50574274 y: -0.27025385 z: 0.85814328
    # predict: match
    test_q = np.array([0.5149363115205982, 2.7840728646265798, 4.284077575957208, -0.5259787760854754, 0.860368012295942, 0.1713596023130354])

    # current bullet environment joint angle
    test_q = np.array([2.08477017189754, 2.94851560508792, 0.9403603166942158, 4.0745507140341335, 0.7758782280277694, 8.224071010650652-2*pi])
    test_q = np.array([2.9654557064658054, 2.9599693283041333, 0.5794236254734035, 3.9531707557671565, 1.8754357233632242, 1.953975539702969])

    test_q = test_q.reshape(6,1)
    # q = Q_all[0,:].reshape(6, 1)
    q = test_q
    Checker = CollisionCheck()
    Checker.simulation2()

    # without checker: [-0.17453293 -0.005       0.07432923 -0.03735876 -0.18331453  0.03045566]
    # with collision checker: [-0.0049198  -0.1234     -0.00917626  0.02173149 -0.0389669  -0.09331467]

    result = QP_abbirb6640(q, np.array([0,0,0,0,0.3,0]), True, Checker)
    print result

    R,pos = fwdkin(test_q,ttype,H,P,n)
    print(pos)
    R,pos = fwdkin(test_q+np.array(result),ttype,H,P,n)
    print(pos)

    # sys.exit(1)

    orien = Quaternion(matrix=R)
    orien = np.array([orien[0], orien[1], orien[2], orien[3]]).reshape(1, 4)


    pos_v = np.zeros((3, 1))
    ang_v = np.array([1,0,0,0])
    dq = np.zeros((int(n),1))

    # joint limits
    lower_limit = np.transpose(np.array([-170*np.pi/180, -65*np.pi/180, -np.pi, -300*np.pi/180, -120*np.pi/180, -2*np.pi]))
    upper_limit = np.transpose(np.array([170*np.pi/180, 85*np.pi/180, 70*np.pi/180, 300*np.pi/180, 120*np.pi/180, 2*np.pi]))

    # inequality constraints
    h = np.zeros((15, 1))
    sigma = np.zeros((13, 1))
    dhdq = np.vstack((np.hstack((np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.hstack((-np.eye(6), np.zeros((6, 1)), np.zeros((6, 1)))), np.zeros((1, 8))))

    # velocities
    w_t = np.zeros((3, 1))
    v_t = np.zeros((3, 1))

    # keyboard controls
    # define position and angle step
    inc_pos_v = 0.01 # m/s
    inc_ang_v = 0.5*np.pi/180 # rad/s

    # optimization params
    er = 0.05
    ep = 0.05
    epsilon = 0 # legacy param for newton iters

    # parameters for inequality constraints
    c = 0.5
    eta = 0.1
    epsilon_in = 0.15
    E = 0.005

    Ke = 1

    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pos,orien,pos_v,ang_v.reshape(1, 4),w_t,v_t,epsilon,inc_pos_v,inc_ang_v,0,er,ep,0)

    dt = 0
    counter = 0

    # dL = 1.0/200.0
    dL = 1.0/5.0
    Lambda = np.linspace(0,1,1.0/dL+1)#np.linspace(0,1,1001)


    pos_b = pos
    R_b = R
    q_b = q
    eu_b = np.array(euler_from_matrix(R_b))

    Joint_all =[]

    Checker = CollisionCheck()
    Checker.simulation2()

    # Q_all: is the path for the arm
    # for iq in range(Q_all.shape[0]-1):
    # here we just define two point, start point and end point:
    my_R,start_pos = fwdkin(test_q,ttype,H,P,n)
    cur_pos = start_pos
    end_pos = start_pos + np.array([0, 0.2, -0.]).reshape((3,1)) # move 10cm in the y direction
    # for iq in range(Q_all.shape[0]-1):
    index = 0
    if 1: # cur_pos != end_pos:
        orien = Quaternion(matrix=my_R)
        iq = Checker.calculateInverseKinematics(end_pos, orien).reshape(6,1)
        iq_copy = iq
        # Checker.update_joint(iq)

        q_a = q_b
        R_a = R_b
        pos_a = pos_b
        eu_a = eu_b
        #        q_a = obj.params['controls']['q']
        #        R_a, pos_a = fwdkin(q_a,ttype,H,P,n)
        #        eu_a = np.array(euler_from_matrix(R_a))

        q_b = iq # Q_all[iq+1,:].reshape(6, 1)

        ## TODO: can not get correct pos_b from fwdkin(q_b). The q_b is from bullet, this function is our own function

        R_b, pos_b = fwdkin(iq,ttype,H,P,n)
        eu_b = np.array(euler_from_matrix(R_b))


        #stop, Closest_Pt, Closest_Pt_env = OpenRAVE_obj.CollisionReport(obj.params['controls']['q'][0],obj.params['controls']['q'][1],obj.params['controls']['q'][2],obj.params['controls']['q'][3],obj.params['controls']['q'][4],obj.params['controls']['q'][5])
        #raw_input('pause')
        for ld in Lambda:

            eu = eu_a*(1-ld) + eu_b*ld
            R_des = euler_matrix(eu[0],eu[1],eu[2])
            R_des = R_des[0:3,0:3]
            #print R_des[0:3,0:3],x_des, eu

            x_des = pos_a*(1-ld) + pos_b*ld
            x_des = np.array([x_des[0][0], x_des[1][0], x_des[2][0]])
            if x_des[1] > 0.5:
                print

            obj.params['controls']['q'] = obj.params['controls']['q'] + obj.params['controls']['dq']#*dt*0.1

            Joint_all.append(np.transpose(obj.params['controls']['q'])[0])
            #print np.transpose(obj.params['controls']['q'])[0]

            pp,RR = fwdkin_alljoints(obj.params['controls']['q'],ttype,obj.params['defi']['H'],obj.params['defi']['P'],obj.params['defi']['n'])

            # parameters for qp
            obj.params['controls']['pos'] = pp[:, -1]

            orien_tmp = Quaternion(matrix=RR[:, :, -1])
            obj.params['controls']['orien'] = np.array([orien_tmp[0], orien_tmp[1], orien_tmp[2], orien_tmp[3]]).reshape(1, 4)

            (Closest_Pt, Closest_Pt_env) = Checker.collision_report()
            # stop, Closest_Pt, Closest_Pt_env = OpenRAVE_obj.CollisionReport(obj.params['controls']['q'][0],obj.params['controls']['q'][1],obj.params['controls']['q'][2],obj.params['controls']['q'][3],obj.params['controls']['q'][4],obj.params['controls']['q'][5])



            # check self-collision
            # if (stop):
            #     print 'robot is about to self-collide.'
            #     #obj.params['controls']['pos_v'] = np.array([0,0,0]).reshape(3, 1)
            #     #obj.params['controls']['ang_v'] = np.array([1,0,0,0]).reshape(1, 4)

            J2C_Joint = Joint2Collision(Closest_Pt, pp)

            J_eef = getJacobian(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])

            v_tmp = Closest_Pt-obj.params['controls']['pos']

            v_tmp2 = (pp[:, -1] - pp[:, -3])
            p_norm2 = norm(v_tmp2)
            v_tmp2 = v_tmp2/p_norm2

            # determine if the closest point is on the panel
            #print norm(v_tmp),np.arccos(np.inner(v_tmp, v_tmp2)/norm(v_tmp))*180/np.pi

            if (norm(v_tmp) < 2.5 and np.arccos(np.inner(v_tmp, v_tmp2)/norm(v_tmp))*180/np.pi < 70):
                # print '---the closest point is on the panel---'
                J2C_Joint = 6
                J = getJacobian3(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'], Closest_Pt,J2C_Joint)
                #J,p_0_tmp = getJacobian2(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'],Closest_Pt,J2C_Joint)

            #if (J2C_Joint < 4):
            else:
                J,p_0_tmp = getJacobian2(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'],Closest_Pt,J2C_Joint)

            #else:
            #   J = getJacobian3(obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'], Closest_Pt,J2C_Joint)


            # update joint velocities
            #axang = quat2axang(obj.params['controls']['ang_v'])

            # desired rotational velocity
            w_skew = logm(np.dot(RR[:,:,-1], R_des.T))
            w = np.array([w_skew[2, 1], w_skew[0, 2], w_skew[1, 0]])
            vr = -Ke*w
            obj.params['controls']['ang_v'] = vr

            # desired linear velocity
            V_desired = np.reshape(Ke*(x_des-pp[:,-1]),[3,1])
            obj.params['controls']['pos_v'] = V_desired

            #print x_des
            #print pp[:,-1]
            #print (x_des-np.reshape(pp[:,-1],[3,1]))
            Q = getqp_H(obj.params['controls']['dq'], J_eef, vr.reshape(3, 1), obj.params['controls']['pos_v'], obj.params['opt']['er'], obj.params['opt']['ep'])

            # make sure Q is symmetric
            Q = 0.5*(Q + Q.T)

            f = getqp_f(obj.params['controls']['dq'],obj.params['opt']['er'], obj.params['opt']['ep'])
            f = f.reshape((8, ))


            # bounds for qp
            if obj.params['opt']['upper_dq_bounds']:
                bound = obj.params['defi']['dq_bounds'][1, :]
            else:
                bound = obj.params['defi']['dq_bounds'][0, :]

            LB = np.vstack((-0.1*bound.reshape(6, 1),0,0))
            UB = np.vstack((0.1*bound.reshape(6, 1),1,1))
            # LB = matrix(LB, tc = 'd')
            # UB = matrix(UB, tc = 'd')

            # inequality constrains A and b
            h[0:6] = obj.params['controls']['q'] - lower_limit.reshape(6, 1)
            h[6:12] = upper_limit.reshape(6, 1) - obj.params['controls']['q']

            dx = Closest_Pt_env[0] - Closest_Pt[0]
            dy = Closest_Pt_env[1] - Closest_Pt[1]
            dz = Closest_Pt_env[2] - Closest_Pt[2]

            """ """
            dist = np.sqrt(dx**2 + dy**2 + dz**2)

            # derivative of dist w.r.t time
            der = np.array([dx/dist, dy/dist, dz/dist])

            """ """
            h[12] = dist - 0.1
            """ """ """ """
            #dhdq[12, 0:6] = np.dot(-der.reshape(1, 3), J_eef2[3:6,:])
            dhdq[12, 0:6] = np.dot(-der[None, :], J[3:6,:])

            sigma[0:12] =inequality_bound(h[0:12], c, eta, epsilon_in, E)
            sigma[12] = inequality_bound(h[12], c, eta, epsilon_in, E)

            A = dhdq
            b = sigma

            A = np.vstack((A, np.eye(8), -np.eye(8)))
            b = np.vstack((b, LB, -UB))
            b = b.reshape((29, ))

            # solve the quadprog problem
            #if not is_pos_def(Q):
            #    print np.linalg.eigvals(Q)
            #    dq_sln = np.zeros((6,1))
            #    raw_input('pause')
            #
            #else:
            dq_sln = quadprog.solve_qp(Q, -f, A.T, b)[0]

            if len(dq_sln) < obj.params['defi']['n']:
                obj.params['controls']['dq'] = np.zeros((6,1))
                V_scaled = 0
                print 'No Solution'
            else:
                obj.params['controls']['dq'] = dq_sln[0: int(obj.params['defi']['n'])]
                obj.params['controls']['dq'] = obj.params['controls']['dq'].reshape((6, 1))
                V_scaled = dq_sln[-1]*V_desired
                vr_scaled = dq_sln[-2]*vr.reshape(3,1)

            V_linear = np.dot(J_eef[3:6,:], obj.params['controls']['dq'])
            V_rot = np.dot(J_eef[0:3,:], obj.params['controls']['dq'])

            iq = Checker.calculateInverseKinematics(x_des, orien).reshape(6,1)
            # Checker.update_joint(obj.params['controls']['q'])
            # print obj.params["controls"]['dq']
            Checker.update_joint(iq)
            # print x_des

            # bullet approach to calculate the jacobian:
            (jac_t, jac_r) = Checker.calculateJacobian(iq_copy)
            print Checker.multiplyJacobian2(obj.params["controls"]['dq'])

            dq = np.concatenate((iq-iq_copy, np.array([[0],[0],[0]])))
            result = dq.reshape(9) * jac_t
            # print '------Scaled desired linear velocity------'
            # # print V_scaled
            #
            # print '------Real linear velocity by solving quadratic programming------'
            # # print V_linear
            # print '------Position--------'
            # print x_des
            # print end_pos
            if index % 100 == 0:
                print
            index += 1
            time.sleep(0.01)

            # print '------Scaled desired angular velocity------'
            # print vr_scaled
            #
            # print '------Real angular velocity by solving quadratic programming------'
            # print V_rot
    raw_input("pause")
    np.savetxt('Joint_all.out', Joint_all, delimiter=',')


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


def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/j2n6s300_driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]
    # goal.angles.joint7 = angle_set[6]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None

currentJointCommand = [0.0] * 6 # number of joints is defined in __main__

def getcurrentJointCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print 'position listener obtained message for joint position. '


def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(6):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])

def getInputVel():
    velocity = np.array([0.0]*6)
    command = raw_input("Please input command => ")
    value = 0.025    # 0.1 is too big
    if command == 'w':
        velocity[4] = value
    elif command == "s":
        velocity[4] = -value
    elif command == 'a':
        velocity[3] = value
    elif command == "d":
        velocity[3] = -value
    elif command == 'q':
        velocity[5] = value
    elif command == "e":
        velocity[5] = -value
    else:
        # print Exception("Unknown command!")
        return getInputVel()
    return velocity


if __name__ == '__main__':
    # main()
    grasp_pose_pub = rospy.Publisher("/object_detection/grasp_pose", geometry_msgs.msg.Pose, queue_size=1)

    # robotics parameter
    ex,ey,ez,n,P,q_ver,H,ttype,dq_bounds = robotParams()

    # initialize the collision check system and move the arm to the initial position
    Checker = CollisionCheck()
    Checker.simulation2()

    # initial joint position
    # test_q = np.array([2.9654557064658054, 2.9599693283041333, 0.5794236254734035, 3.9531707557671565, 1.8754357233632242, 1.953975539702969])
    # test_q = np.array([108.419143677, 226.019287109, 37.444770813, 159.409164429, 114.613708496, 507.272796631]) * pi /180
    q = Checker.getKinovaJointState("radius").reshape(6,1)

    # orientation = np.array([0, sqrt(2)/2, sqrt(2)/2, 0]).reshape((1,4))
    orientation = np.array([-0.0046213939786, 0.725189089775, 0.684360861778, 0.0756933689117])


    while 1:
    # for i in range(5):
        velocity = getInputVel()
        result = np.array(QP_abbirb6640(q, velocity, True, Checker)).astype(float)
        # print result

        R,pos1 = fwdkin(q,ttype,H,P,n)
        # print "Position1 is {}".format(pos)
        # print(pos)  # [0.3159], [0.3044]

        q = q + np.array(result).reshape((6,1))
        # print np.array(result).reshape((6,1))
        R,pos2 = fwdkin(q,ttype,H,P,n)
        # print "q and pos is {}, {}".format(q, pos)
        print (pos2 - pos1).reshape((3,))
        Checker.update_joint(np.array(result), "relative")

        # q = Checker.calculateInverseKinematics(pos, orientation).reshape(6,1)
        # print pos.reshape((3,)), orientation
        # cartesian_pose_client(pos.reshape((3,)), orientation)
        # print q
        joint_angle_client(q.reshape((6,))*180/pi)
        # break
        time.sleep(0.01)


    # R,pos = fwdkin(test_q+np.array(result),ttype,H,P,n)
        # print(pos)
