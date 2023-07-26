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
import quaternion

def qt2H(r, t):
    # quaternion and translation to homogenous transformation matrix
    # r is (4,) in [x,y,z,w] order
    # t is (3,)
    q = R.from_quat(r)
    rotm = np.array(q.as_matrix())
    H = ([[rotm[0,0], rotm[0,1], rotm[0,2], t[0]],
          [rotm[1,0], rotm[1,1], rotm[1,2], t[1]],
          [rotm[2,0], rotm[2,1], rotm[2,2], t[2]],
          [0,          0,         0,         1   ]])

    return H


def ds_cap(pressure_sens, t_W_B, r_W_B, t_B_C, r_B_C, t_C_P, r_C_P):
    # print(f"\npositioning_start: {pressure_sens}, {t_C_P}, {r_C_P}")

    # print(f"r_W_B is: {r_W_B}")
    H_W_B = qt2H(r_W_B, t_W_B)
    # print(f"H_W_B is: {H_W_B}")     
    H_B_C = qt2H(r_B_C, t_B_C)
    # print(f"H_B_C is: {H_B_C}") 
    # H_W_C = np.matmul(H_W_B, H_B_C)
    H_W_C = np.array(H_W_B) @ np.array(H_B_C)
    # print(f"H_W_C is: {H_W_C}") 
    x_W_C = H_W_C[0,3]
    y_W_C = H_W_C[1,3]
    z_W_C = H_W_C[2,3]

    # H_W_P = H_W_C * H_C_P
    
    H_C_P = qt2H(r_C_P, t_C_P)
    H_W_P = np.matmul(H_W_C, H_C_P)

    # print(f"positioning_mid  : {pressure_sens}, {t_C_P}, {r_C_P}")
    x_W_P = H_W_P[0,3]
    y_W_P = H_W_P[1,3]
    z_W_P = H_W_P[2,3]

    # tf_W_P = TransformBroadcaster()
    # tf_W_P.sendTransform((x_W_P, y_W_P, z_W_P), (0,0,0,1), rospy.Time.now(), 'PPPPPPPPPPPPPPPP', 'world')

    # if z_W_C == z_W_P:
        # z_W_C = 1000000000000 # Avoding being divided by 0
    # else:
    k = (pressure_sens - z_W_P) / (z_W_C - z_W_P)
    x_W_M = x_W_P + (x_W_C - x_W_P) * k
    y_W_M = y_W_P + (y_W_C - y_W_P) * k
    z_W_M = z_W_P + (z_W_C - z_W_P) * k
        
        # tf_W_M = TransformBroadcaster()
        # tf_W_M.sendTransform((x_W_M, y_W_M, z_W_M),  (0,0,0,1), rospy.Time.now(), 'MMMMMMMMMMMMMM', 'world')

    t_W_M = (x_W_M, y_W_M, z_W_M)

    # print(f"positioning_end  : {pressure_sens}, {t_C_P}, {r_C_P}\n")
    # return t_W_M, pressure_sens, t_W_B, r_W_B, t_C_P, r_C_P
    return t_W_M

def unitq(axis,angle):
    # creat a unit quaternion
    axis_norm = axis/np.linalg.norm(axis)
    im = np.sin(angle/2) * axis_norm
    r = (im[0], im[1], im[2], np.cos(angle/2))
    return r



def spoof_cam(t_W_B, r_W_B, t_B_C, r_B_C, H_W_M):
    H_W_B = qt2H(r_W_B, t_W_B)           
    H_B_C = qt2H(r_B_C, t_B_C)
            
    H_W_C = np.matmul(H_W_B, H_B_C)
    
    # H_C_M = H_W_C_^-1 * H_W_M
    H_C_M_sim = np.matmul(inv(np.array(H_W_C)), np.array(H_W_M))

    # r_wc = R.from_matrix([[H_W_C[0,0], H_W_C[0,1],H_W_C[0,2]],
    #                               [H_W_C[1,0], H_W_C[1,1],H_W_C[1,2]],
    #                               [H_W_C[2,0], H_W_C[2,1],H_W_C[2,2]]])
    # r_W_C_forward = r_wc.as_quat()

    # r_cm = R.from_matrix([[H_C_M_sim[0,0], H_C_M_sim[0,1],H_C_M_sim[0,2]],
    #                       [H_C_M_sim[1,0], H_C_M_sim[1,1],H_C_M_sim[1,2]],
    #                       [H_C_M_sim[2,0], H_C_M_sim[2,1],H_C_M_sim[2,2]]])
    # r_C_M_sim = r_cm.as_quat()
            
    H_C_P = H_C_M_sim
    tx_C_P = H_C_P[0,3] / H_C_P[2,3]
    ty_C_P = H_C_P[1,3] / H_C_P[2,3]
    tz_C_P = H_C_P[2,3] / H_C_P[2,3]
    t_C_P = (tx_C_P, ty_C_P, tz_C_P)

    return t_C_P 


 
def quaternion_multiply(Q0,Q1):
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
 
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion


def imu_alignment(r_world_sb, r_arb_imu, r_imu_b):
    r_w_imu = quaternion_multiply(r_world_sb, r_arb_imu)
    r_w_b = quaternion_multiply(r_w_imu, r_imu_b)

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

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)


def get_ekf(H,Q,jacobianH):
        # Ground Truth
        x_true = get_status(x_true)

        # [step1] prediction
        x_bar = get_status(x_hat)

        # jacobian matrix
        jacobianF = get_jacobianF(x_bar)

        # pre_covariance
        P_bar = (jacobianF @ P @ jacobianF.T) + Q

        # observation
        w = np.random.multivariate_normal([0.0, 0.0], R, 1).T
        y = (H @ x_true) + w

        # [step2] update the filter
        s = (H @ P_bar @ H.T) + R
        K = (P_bar @ H.T) @ np.linalg.inv(s)

        # eastimation
        e = y - (jacobianH @ x_bar)
        x_hat = x_bar + (K @ e)

        # post_covariance
        I = np.identity(x_hat.shape[0])
        P = (I - K @ H) @ P_bar

        return x_true, y, x_hat, P


def get_status(x,gyro_angular_vel,dts):
    # get status
    tri = get_trigonometrxic(x)
    Q = np.array([[1, tri[0,1]*tri[1,2], tri[0,0]*tri[1,2]], [0, tri[0,0], -tri[0,1]]])
    x =  x + (Q @ gyro_angular_vel) * dts
    return x


def get_jacobianF(x,gyro_angular_vel,dts):
    # get jacobian matrix of F
    g = gyro_angular_vel
    
    tri = get_trigonometrxic(x)
    jacobianF = np.array([[1.0+(tri[0,0]*tri[1,2]*g[1][0]-tri[0,1]*tri[1,2]*g[2][0])*dts, (tri[0,1]/tri[1,0]/tri[1,0]*g[1][0]+tri[0,0]/tri[1,0]/tri[1,0]*g[2][0])*dts], 
                          [-(tri[0,1]*g[1][0]+tri[0,0]*g[2][0])*dts, 1.0]])
    return jacobianF


def get_trigonometrxic(x):
    # get trigonometrxic of roll&pitch
    return np.array([[np.cos(x[0][0]), np.sin(x[0][0]), np.tan(x[0][0])], [np.cos(x[1][0]), np.sin(x[1][0]), np.tan(x[1][0])]])

def tilting_plus_slamyaw(roll,pitch,slamyaw,r_i2b):
    
    eul = (roll, pitch, slamyaw)
    r_w2i = np.array([tf.transformations.quaternion_from_euler(eul[0], eul[1], eul[2])])
    q_w2i = np.quaternion(r_w2i[0][3], r_w2i[0][0], r_w2i[0][1], r_w2i[0][2])
    q_i2b = np.quaternion(r_i2b[3], r_i2b[0], r_i2b[1], r_i2b[2])
    q_w2b = q_w2i * q_i2b
    r_w2b = np.array([q_w2b.x, q_w2b.y, q_w2b.z, q_w2b.w])
    # print(eul)
    return r_w2b