#!/usr/bin/python

# Forward kinematics.
# Qingyuan Jiang Feb 2018

import numpy as np
from math import sqrt,cos,sin,tan,pi
from functools import partial
# from collision import Segment

# Original Point
    # (0,0,0), alpha, a, d, theta
# joint1: Position(0,0,0.15675)
# joint2: Position(0,0,0.2755)
# joint3: Position(0,0,0.6855)
# joint4: Position(0,0.0097828,0.8928)
# joint5: Position(0,0.044032,0.95863)
# joint6: Position(0,0.11769,0.96839)
# tip : Position(0,0.25424,0.91870)
pos = [[0,0,0],[0,0,0.15675],[0,0,0.2755],[0,0,0.6855],[0,0.0097828,0.8928],[0,0.044032,0.95863],[0,0.11769,0.96839],[0,0.25424,0.91870]]

class link:
    # private upper_joint_Position
    # private lower_joint_Position
    # public function link_length
    def __init__(self,alpha,a,d,theta0,outer=False):
        # self.link_length = link_length
        self.alpha = alpha
	self.a = a
	self.d = d
	self.theta0 = theta0
	self.outer=outer
    def get_rotation_matrix(self,theta):
	if self.outer == True:
	    return self.transfer_matrix(theta)
	else:
	    return DH_matrix(self.alpha,self.a,self.d,theta-self.theta0)

    def print_link(self):
	print "##### PARAMETERS OF THE LINK #####"
	print "alpha: ", self.alpha
	print "a: ", self.a
	print "d: ", self.d
	print "theta0: ", self.theta0
    def test_rotation_matrix(self,theta):
	print "##### Test of Rotation Matrix #####"
 	print self.get_rotation_matrix(theta)
	print " "

def DH_matrix(alpha,a,d,theta):
    return np.matrix([[cos(theta),-cos(alpha)*sin(theta),sin(alpha)*sin(theta),a*cos(theta)],[sin(theta),cos(alpha)*cos(theta),-sin(alpha)*cos(theta),a*sin(theta)],[0,sin(alpha),cos(alpha),d],[0,0,0,1]])

#################################################################
############### INITIAL JACO-ARM INFORMATION HERE ###############
#################################################################

base = link(pi,0,(pos[1][2]-pos[0][2]),0)
link1 = link(pi/2,0,-(pos[2][2]-pos[1][2]),pi)
link2 = link(pi,(pos[3][2]-pos[2][2]),0,3*pi/2)
link3 = link(pi/2,(pos[4][2]-pos[3][2]),-(pos[4][1]-pos[3][1]),-pi/2,True)
link4 = link(-55*pi/180,0,-(pos[5][2]-pos[4][2]),pi,True)
link5 = link(-55*pi/180,0,-0.065935,pi,True)
link6 = link(pi,0,-0.14531,0,True)
links = [base,link1,link2,link3,link4,link5,link6]

def transfer_matrix_link3(self,theta):
    return np.matrix([[cos(theta+pi/2),0,sin(theta+pi/2),-sin(theta+pi/2)*(pos[4][2]-pos[3][2])],[cos(theta),0,sin(theta),-sin(theta)*(pos[4][2]-pos[3][2])],[0,1,0,-(pos[4][1]-pos[3][1])],[0,0,0,1]])

def transfer_matrix_link4(self,theta):
    m = np.matrix([[-cos(theta),sin(35*pi/180)*sin(theta),cos(35*pi/180)*sin(theta),-sin(theta)*(pos[5][1]-pos[4][1])],
		   [-sin(theta),-sin(35*pi/180)*cos(theta),-cos(35*pi/180)*cos(theta),cos(theta)*(pos[5][1]-pos[4][1])],
		   [0,-cos(35*pi/180),sin(35*pi/180),-(pos[5][2]-pos[4][2])],
		   [0,0,0,1]])
    # print "Matrix m: ", m
    return m

def transfer_matrix_link5(self,theta):
    link_1 = link(-35*pi/180,0,-(pos[6][2]-pos[5][2])/cos(55*pi/180),pi)
    link_2 = link(-20*pi/180,0,-((pos[6][1]-pos[5][1])-(pos[6][2]-pos[5][2])*tan(55*pi/180)),0)
    # print link_1.get_rotation_matrix(pi)
    # print link_2.get_rotation_matrix(0)
    # print link_1.get_rotation_matrix(theta)*link_2.get_rotation_matrix(0)
    return link_1.get_rotation_matrix(theta)*link_2.get_rotation_matrix(0)

def transfer_matrix_link6(self,theta):
    link_1 = link(20*pi/180,0,-(pos[6][2]-pos[7][2])/cos(70*pi/180),pi)
    link_2 = link(160*pi/180,0,-((pos[7][1]-pos[6][1])-(pos[6][2]-pos[7][2])*tan(70*pi/180)),0)
    # print link_1.get_rotation_matrix(pi)
    # print link_2.get_rotation_matrix(0)
    # print link_1.get_rotation_matrix(theta)*link_2.get_rotation_matrix(0)
    return link_1.get_rotation_matrix(theta)*link_2.get_rotation_matrix(0)

link3.transfer_matrix = partial(transfer_matrix_link3, link3)
link4.transfer_matrix = partial(transfer_matrix_link4, link4)
link5.transfer_matrix = partial(transfer_matrix_link5, link5)
link6.transfer_matrix = partial(transfer_matrix_link6, link6)

#################################################################


def joint_fk(number,configure):
    matrix = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    if number == 0:
	return matrix
    else:
	configure = [0] + configure
        for i in xrange(number):
	    # print "Multiply links: i ", i
            matrix = matrix*links[i].get_rotation_matrix(configure[i])
        return matrix

def get_joint_position(number,configure):
    matrix = joint_fk(number,configure)
    # position = np.matrix([[matrix[0,3],matrix[1,3],matrix[2,3]]])
    return [matrix[0,3],matrix[1,3],matrix[2,3]]

def forward_kinematics(configure,number = 6):
    return get_joint_position(number,configure)


############### TEST FUNCTIONS BELLOW ###############

def test_ForwardKinematics(configure):
    print "########## Test for Forward Kinematics ##########"
    for i in xrange(8):
        print "Matrix For Joint No.i: ", i
        print joint_fk(i,configure)
	# print get_joint_position(i,configure)
	print " "

def test_link3():
    print "########## Test for Link3 ##########"
    print "link3.get_rotation_matrix"
    print link3.transfer_matrix(pi)
    print link3.outer
    print link3.get_rotation_matrix(pi)
    print ' '

def test_link4():
    print "########## Test for Link4 ##########"
    print "link4.get_rotation_matrix"
    print link4.transfer_matrix(pi)
    print link4.outer
    print link4.get_rotation_matrix(pi)
    print ' '

def test_link5():
    print "########## Test for Link5 ##########"
    print "link5.get_rotation_matrix"
    # print link5.transfer_matrix(pi)
    print link5.outer
    print link5.get_rotation_matrix(pi)
    print ' '

def test_link6():
    print "########## Test for Link6 ##########"
    print "link6.get_rotation_matrix"
    # print link6.transfer_matrix(pi)
    print link6.outer
    print link6.get_rotation_matrix(pi)
    print ' '

############### MAIN FUNCTION ###############

if __name__ == '__main__':
    
    # test_link3()
    # test_link4()
    # test_link5()
    test_link6()
    # Test for construction of each link
    #for link in links:
	# link.print_link()
	#link.test_rotation_matrix(pi)

    # link1.print_link

    # Test for Forward Kinematics.
    configure = [pi,pi,pi,pi,pi,pi]  
    test_ForwardKinematics(configure)






















