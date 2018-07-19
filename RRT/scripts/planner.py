#!/usr/bin/python

import numpy as np


joint_limits = [
    [-2*np.pi, 2*np.pi],  # make joint 1 smaller from real
    [47/180*np.pi, 266/180*np.pi],
    [19*np.pi/180, 322*np.pi/180],
    [-2*np.pi, 2*np.pi],
    [-2*np.pi, 2*np.pi],
    [-2*np.pi, 2*np.pi]]

'''
# Real Version
joint_limits = [
    [-10000*np.pi/180, 20000*np.pi/180],  # make joint 1 smaller from real
    [47*np.pi/180, 266/180*np.pi],
    [19*np.pi/180, 322*np.pi/180],
    [-10000*np.pi/180, 20000*np.pi/180],
    [-10000*np.pi/180, 20000*np.pi/180],
    [-10000*np.pi/180, 20000*np.pi/180]] 
'''

if __name__ == '__main__':
    q_rand = [ np.random.uniform(joint_limits[0][0], joint_limits[0][1]), 
                   np.random.uniform(joint_limits[1][0], joint_limits[1][1]),
		   np.random.uniform(joint_limits[2][0], joint_limits[2][1]),
		   np.random.uniform(joint_limits[3][0], joint_limits[3][1]), 
		   np.random.uniform(joint_limits[4][0], joint_limits[4][1]),
		   np.random.uniform(joint_limits[5][0], joint_limits[5][1]),]
    print "q_rand: "
    print q_rand
