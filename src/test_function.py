#!/usr/bin/env python3
import tf
import rospy
import math
import tf
import numpy as np
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
import datetime
import time
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import sys
import numpy.matlib as npm
import numpy
import numpy as np


# Create a quaternion from Euler angles
q = R.from_euler('xyz', [0, 0, 0], degrees=True)

# print("Quaternion:", q.as_quat())


def weightedAverageQuaternions(Q, w):
    # Average multiple quaternions with specific weights
    # The weight vector w must be of the same length as the number of rows in the
    # quaternion maxtrix Q
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))
    weightSum = 0

    for i in range(0,M):
        q = Q[i,:]
        A = w[i] * numpy.outer(q,q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A


    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)

    # # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)



def main():

    # Create two quaternions
    q1 = R.from_euler('xyz', [0, 0, 0], degrees=True)
    q2 = R.from_euler('xyz', [0, 0, 90], degrees=True)
    q3 = R.from_euler('xyz', [0, 0, 90], degrees=True)

    q4 = q3.inv()
    # Multiply the quaternions
    q_result = q1 * q2 * q3

    # print("First quaternion:", q1.as_quat())
    # print("Second quaternion:", q2.as_quat())
    # print("Result of multiplication:", q_result.as_quat())


    a = weightedAverageQuaternions(np.array(([0.7071,0,0,0.7071], [0,0,0,1])), np.array((1/2,1/2)))
    # print(a[0] + a[2])
    print(q4)

if __name__ == '__main__':
    main()