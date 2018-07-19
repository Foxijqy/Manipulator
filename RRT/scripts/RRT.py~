#! usr/bin/python

# RRT for obstacle trajectory planning
# Qingyuan Jiang Feb 2018

from math import pi,sqrt
from kinematics import forward_kinematics
import numpy as np
from planner import joint_limits
from collision import Obstacle, in_collision, path_in_collision

# Parameters Initial Here
XDIM = 1
YDIM = 1
ZDIM = 1
EPSILON = 10000	# (rad)
TARGET_RADIUS = 0.06 	# (meter)
NUMNODES = 5000
NIter = 10000

center1 = [0.225,0.2,0.2]
center2 = [-0.325,0.375,0.2]
ob1 = Obstacle(center1,0.2,0.2,0.4)
ob2 = Obstacle(center2,0.2,0.2,0.4)
obs = [ob1,ob2]

config0 = [pi,pi,pi,pi,pi,pi]
target_x = [0,0.6,0.125]

def C_dist(configure1, configure2):
    sigma = 0
    for i in xrange(6):
	sigma = sqrt((configure1[i]-configure2[i])*(configure1[i]-configure2[i]) + sigma*sigma)
    return sigma

def S_dist(point1,point2):
    dx = point2[0]-point1[0]
    dy = point2[1]-point1[1]
    dz = point2[2]-point1[2]
    return sqrt(dx*dx+dy*dy+dz*dz)

class Node:
    def __init__(self, config, parent_id = None):
	self.config = config
	self.parent_id = parent_id


def in_workspace(config):
    x = forward_kinematics(config)
    return abs(x[0])<=XDIM and abs(x[1])<=YDIM and abs(x[2])<=ZDIM



def find_nearest_node(nodes,config):
    min_index = 0
    for i, node in enumerate(nodes):
	if C_dist(node.config,config) < C_dist(config,nodes[min_index].config):
	    min_index = i
    return min_index


def step_from_toward(config1,config2):
    if C_dist(config1,config2) < EPSILON:
	return config2
    else:
	lamda = EPSILON/C_dist(config1,config2)
	config_new = []
	for i in xrange(6):
	    config_new.append(config1[i]+lamda*(config2[i]-config1[i]))
	return config_new


# trace from "node" back to the root of the tree represented by "nodes"
def backtrace(nodes, node):
    plan = []
    curr_node = node
    while True:
        plan.append(curr_node.config)
        if curr_node.parent_id is None:
            break
        curr_node = nodes[ curr_node.parent_id ]        
    plan.reverse()
    return plan

def generate_random_config():
    config_rand = []
    for i in xrange(6):
	config_rand.append(np.random.uniform(joint_limits[i][0],joint_limits[i][1]))

    return config_rand



########## Algorithm RRT Implementation ##########

def RRT(target_x, config0, NIter=10000):
    
    nodes = [Node(config0)]
    start_x = forward_kinematics(config0)

    for i in xrange(NIter):
	config_random = generate_random_config()
	nearest_node_index = find_nearest_node(nodes,config_random)
	config_new = step_from_toward(nodes[nearest_node_index].config, config_random)

	# Use for test here
        '''
	print "New Configuration: ", config_new
	print "in_workspace(config_new) ", in_workspace(config_new)
	print "in_collision(config_new) ", in_collision(config_new,obs)
	print "path_in_collision(config_new) ", path_in_collision(config_new, nodes[nearest_node_index].config, obs)
	'''

	if in_workspace(config_new) and not in_collision(config_new, obs) and not path_in_collision(config_new, nodes[nearest_node_index].config, obs):
	    new_node = Node(config_new, nearest_node_index)
	    nodes.append(new_node)
	    
	    print "Add new node, list length: ", len(nodes)
	    # print "Distance: ", S_dist(forward_kinematics(config_new),target_x)

	    if S_dist(forward_kinematics(config_new),target_x) < TARGET_RADIUS:
		plan = backtrace(nodes, new_node)
		return plan
    # for loop end
    return None # no plan found


def test_Initial():

    print " ########## Initial State ########## "
    print " forward_kinematics(config0): "
    print " ",forward_kinematics(config0)
    print " in_collision(config0,obs): ", in_collision(config0,obs)

def test_RRT():
    
    print " ########## Start Running RRT ########## "
    plan = RRT(target_x,config0,NIter)
    if plan is None:
        print 'no plan found in %d iterations' % NIter
        return
    else:
        print 'found 1 plan', plan
        return


if __name__ == '__main__':
    
    test_Initial()
    test_RRT()




















