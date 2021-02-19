#!/usr/bin/env python

import numpy as np
import math
from math import pi
import tf
from tf.transformations import rotation_matrix, quaternion_matrix, quaternion_from_matrix, inverse_matrix, euler_from_quaternion

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

# position
my_rotm =  np.dot(rotm([1, 0, 0], 0.5*pi), rotm([0, 1, 0], -0.5*pi) )  # axang2tform([0 1 0 -0.5*pi]);

v = [0, 0, 0.3]
# axis = [4, 4, 1]
# theta = 1.2 
v = np.dot(my_rotm, v)
# print() 
# my_rotm = 
# v = np.dot(my_rotm, v)
print(v)
# [ 2.74911638  4.77180932  1.91629719]

# orientation
end2tag = tf.transformations.quaternion_matrix([0.52, -0.51, 0.48, -0.47])
my_rotm = np.dot(rotation_matrix(0.5*pi, (1, 0, 0)), rotation_matrix(-0.5*pi, (0, 1, 0)))
result =  np.dot( inverse_matrix(my_rotm), end2tag)# axang2tform([0 1 0 -0.5*pi]);
qs = tf.transformations.quaternion_from_matrix(result)
print qs
euler_angle = euler_from_quaternion([qs[1], qs[2], qs[3], qs[0]])
print euler_angle
