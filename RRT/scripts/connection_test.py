#!/usr/bin/env python

# Subscriber and Publisher asking robot following points

import rospy
from std_msgs.msg import Float64
from collision import Obstacle
from math import pi
from RRT import RRT

'''
# Parameters Initial Here
XDIM = 1
YDIM = 1
ZDIM = 1
EPSILON = 10000	# (rad)
TARGET_RADIUS = 0.05 	# (meter)
NIter = 10000

center1 = [0.225,0.2,0.2]
center2 = [-0.325,0.375,0.2]
ob1 = Obstacle(center1,0.2,0.2,0.4)
ob2 = Obstacle(center2,0.2,0.2,0.4)
obs = [ob1,ob2]

config0 = [pi,pi,pi,pi,pi,pi]
target_x = [0,0.6,0.125]
'''

######################################################################
# Initialization of ROS elements.
# print("Initializing...")
rospy.init_node('Trajector', anonymous=False)
exec_pubs = [rospy.Publisher('/joint1_controller/command',Float64, queue_size=1),rospy.Publisher('/joint2_controller/command',Float64, queue_size=1),rospy.Publisher('/joint3_controller/command',Float64, queue_size=1),rospy.Publisher('/joint4_controller/command',Float64, queue_size=1),rospy.Publisher('/joint5_controller/command',Float64, queue_size=1),rospy.Publisher('/joint6_controller/command',Float64, queue_size=1)]
######################################################################

def exec_joints(exec_pubs, q):
    print "exec_joints: ", q
    for i in xrange(6):
	exec_pubs[i].publish(Float64(q[i]))

plan = [[pi/6,pi/6,pi/6,pi/6,pi/6,pi/6],[pi/3,pi/3,pi/3,pi/3,pi/3,pi/3],[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2],[2*pi/3,2*pi/3,2*pi/3,2*pi/3,2*pi/3,2*pi/3],[pi,pi,pi,pi,pi,pi]]
def main():
    print " ########## Start Running RRT ########## "
    # plan = RRT(target_x,config0,NIter)
    '''
    if plan is None:
        print 'no plan found in %d iterations' % NIter
        return
    else:
        print 'found 1 plan', plan
    '''
    '''
    index = 0
    while not rospy.is_shutdown():
	if index == len(plan):
	    print "Planning Finish"
	    break
	exec_joints(exec_pubs,plan[index])
	index = index + 1
	rospy.sleep(10)
    '''
    for config in plan:
	exec_joints(exec_pubs,config)
	rospy.sleep(10)
    return

# if python says run, then we should run
if __name__ == '__main__':
    main()

























