#!/usr/bin/env python

# Kinematics used for Manipulators.
# Qingyuan Jiang. 2018 Aug.

# Quaternion to Rotation Matrix
# Rotation Matrix to Quaternion.
# Quaternion to Rotation Matrix

from scipy.linalg import norm
import numpy as np

def Quaternion2Rotation(q):
	
	q = q/norm(q)
	
	qw = q[3]
	qx = q[0]
	qy = q[1]
	qz = q[2]
	
	r = np.identity(3)
	r[0][0] = 1 - 2*pow(qy,2) - 2*pow(qz,2)
	r[0][1] = 2*qx*qy - 2*qz*qw
	r[0][2] = 2*qx*qz + 2*qy*qw
	
	r[1][0] = 2*qx*qy + 2*qz*qw
	r[1][1] = 1 - 2*pow(qx,2) - 2*pow(qz,2)
	r[1][2] = 2*qy*qz - 2*qx*qw
	
	r[2][0] = 2*qx*qz - 2*qy*qw
	r[2][1] = 2*qy*qz + 2*qx*qw
	r[2][2] = 1 - 2*pow(qx,2) - 2*pow(qy,2)
	
	return r
	

def Rotation2Quaternion(r):

	qw = math.sqrt(1 + r[1][1] + m[2][2] + m[3][3])/2.
	qx = (r[3][2] - r[2][3])/(4*qw)
	qy = (r[1][3] - r[3][1])/(4*qw)
	qz = (r[2][1] - r[1][2])/(4*qw)
	q = np.array([qx,qy,qz,qw])
	
	return q
	

