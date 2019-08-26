#!/usr/bin/env python

# Qingyuan Jiang
# Test for ROS system.

import rospy
from math import pi
from std_msgs.msg import Float32MultiArray

node = rospy.init_node('Trajector',anonymous = False)
act_pub = rospy.Publisher('joint_controller/command',Float32MultiArray,queue_size=1)

def execute(config):
	rospy.loginfo("Exc: %s",config)
	array_pub = Float32MultiArray(data=config)
	act_pub.publish(array_pub)
	

def main():
	
	
	config_list = []
	for i in xrange(3):
		config_list.append([pi,pi,pi,pi,pi,pi])
		config_list.append([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2])
		config_list.append([pi/3,pi/3,pi/3,pi/3,pi/3,pi/3])
		config_list.append([pi/4,pi/4,pi/4,pi/4,pi/4,pi/4])
		config_list.append([pi/3,pi/3,pi/3,pi/3,pi/3,pi/3])
		config_list.append([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2])
		config_list.append([pi,pi,pi,pi,pi,pi])
	
	for i in xrange(len(config_list)):
		execute(config_list[i])
		rospy.sleep(3)
		
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
