#!/usr/bin/python

# Forward kinematics.
# Qingyuan Jiang Feb 2018
# Version: 2018-2-9

import numpy as np
from math import sqrt,cos,sin,pi
from kinematics import get_joint_position, forward_kinematics
# from class_Node import Node

class Segment:
    # public vertex1: position--numpy.matrix 
    def __init__(self,vertex1,vertex2):
	self.vertex1 = vertex1
	self.vertex2 = vertex2
	self.NUMBER = 100
    def length(self):
        dx = self.vertex1[0]-self.vertex2[0]
	dy = self.vertex1[1]-self.vertex2[1]
	dz = self.vertex1[2]-self.vertex2[2]
	return sqrt(dx*dx+dy*dy+dz*dz)
    def traverse(self):
	pointlist = []
	count = 0.
	while count<self.NUMBER:
            dx = self.vertex1[0]-self.vertex2[0]
	    dy = self.vertex1[1]-self.vertex2[1]
	    dz = self.vertex1[2]-self.vertex2[2]
	    new_x = self.vertex2[0] + count/self.NUMBER*dx
	    # print "new_x: ", new_x
	    new_y = self.vertex2[1] + count/self.NUMBER*dy
	    new_z = self.vertex2[2] + count/self.NUMBER*dz
	    new_point = [new_x,new_y,new_z]
	    # print "new Point: ", new_point
	    pointlist = pointlist + [new_point]
	    count = count+1
	return pointlist
	
class Obstacle:
    def __init__(self,center,length,width,high):
	self.center = center
	self.length = length	# length is for range x
	self.width = width	# width is for range y
	self.high = high	# high is for range z
    def isIn_point(self,point):
	x = point[0]
	y = point[1]
	z = point[2]
	return x<self.center[0]+self.length/2 and x>self.center[0]-self.length/2 and y>self.center[1]-self.width/2 and y<self.center[1]+self.width/2 and z>self.center[2]-self.high/2 and z<self.center[2]+self.high/2


def isIntersect(segment, obstacles):
    for ob in obstacles:
	pointlist = segment.traverse()
	for point in pointlist:
	    if ob.isIn_point(point):
		return True
    return False

def form_robotics(configure):
    segs = []
    endpoint = [0,0,0]
    i = 1
    while i<6:
	joint_position = get_joint_position(i,configure)
        seg =  Segment(endpoint,joint_position)
	segs.append(seg)
	endpoint = joint_position
	i = i+1
    return segs

def in_collision(configure, obstacles):
    model = form_robotics(configure)
    for seg in model:
        if isIntersect(seg,obstacles):
	    return True
    return False

def path_in_collision(q1, q2, obstacles):
    endpoint1 = forward_kinematics(q1)
    endpoint2 = forward_kinematics(q2)
    seg = Segment(endpoint1,endpoint2)
    return isIntersect(seg,obstacles)


if __name__ == '__main__':
    ################### TEST FOR CLASS SEGMENT #######################
    print "############ TEST FOR CLASS SEGMENT: ######################"
    vertex1 = [0,1,2]
    vertex2 = [3,4,5]
    print "vertex1: "
    print vertex1
    print "vertex2: "
    print vertex2
    seg = Segment(vertex1,vertex2)
    print seg.length()
    print len(seg.traverse())
    #for point in seg.traverse():
	#print point

    ################### TEST FOR CLASS OBSTACLE ######################
    print "############ TEST FOR CLASS OBSTACLE: #####################"
    center1 = [-1,-1,-1]
    center2 = [4,4,4]
    point = [3.5,4,4.5]
    ob1 = Obstacle(center1,2,2,2)
    ob2 = Obstacle(center2,2,2,2)
    ob3 = Obstacle(center2,10,10,10)
    obs = [ob1,ob2,ob3]
    print ob1.isIn_point(point)
    print ob2.isIn_point(point)
    print ob3.isIn_point(point)

    ################### TEST FOR FUNCTION: isIntersect ###############
    print "############ TEST FOR FUNCTION: isIntersect ###############"
    print isIntersect(seg,obs)
    vertex3 = [0,0,0]
    seg2 = Segment(vertex1,vertex3)
    print isIntersect(seg2,obs)
    obs2 = [ob1,ob2]
    print isIntersect(seg2,obs2)

    ################### TEST FOR FUNCTION: form_robotics #############
    print "############ TEST FOR FUNCTION: form_robotics #############"
    configure = [0,0,0,0,0,0]
    model = form_robotics(configure)
    for seg in model:
	# print seg.vertex1, seg.vertex2, seg.length()
	print seg.length()

    ################### TEST FOR FUNCTION: in_collision ##############
    print "############ TEST FOR FUNCTION: in_collision ##############"
    print in_collision(configure,obs)


















