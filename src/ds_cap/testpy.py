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
import pandas as pd

df = pd.read_csv('/home/xl/bag666/cap_online666.csv')
slam_df = pd.read_csv('/home/xl/bag666/cap_slam.csv')
imu_df = pd.read_csv('/home/xl/bag666/cap_imu.csv')
t_c2p_df = pd.read_csv('/home/xl/bag666/cap_t_c2p.csv')
depth_df = pd.read_csv('/home/xl/bag666/cap_depth.csv')

cap_online = df.values
slam = slam_df.values
imu = imu_df.values

t_c2p = t_c2p_df.values
depth = depth_df.values


n_samples = len(slam)

# Initialize an empty list for t_W_M
t_W_M = []

def qt2H(r, t):
    # quaternion and translation to homogenous transformation matrix
    # r is (4,)
    # t is (3,)
    q = R.from_quat(r)
    rotm = np.array(q.as_matrix())
    # print(rotm)
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

z_W_M = depth[:,1]
q_W_B = imu[:,4:8]
# print(q_W_B.shape)
t_W_B_slam = slam[:,4:7]
q_B_C = np.array([0.7071, -0.7071, 0, 0])
# print(q_B_C.shape)
t_B_C = np.array([0,0,-0.22])
p_cp = t_c2p[:,4:7]
r_C_P = np.array([0,0,0,1])

# qt2H(q_W_B[3,:],t_W_B_slam[3,:])
for i in range(n_samples):
    t_W_M_i = ds_cap(z_W_M[i], t_W_B_slam[i,:], q_W_B[i,:], t_B_C, q_B_C, p_cp[i,:], r_C_P)
    t_W_M.append(t_W_M_i)

cap_offline_python = pd.DataFrame(t_W_M, columns=['x','y','z'])

# 保存为 csv 文件
cap_offline_python.to_csv('/home/xl/bag666/offlinepython.csv', index=False)