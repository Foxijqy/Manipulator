#!/usr/bin/env python

# System Establish for Manipulator
# Qingyuan Jiang

import os
import rospy
import numpy as np
import threading

from kinematics import Quaternion2Rotation
from geometry_msgs.msg import PoseStamped


class Mico():
	
	def __init__(self):
		
		self.setup_param()
		# Ros setup.
		self.node = rospy.init_node('Mico',anonymous=False)
		self.subscriber = rospy.Subscriber("m1n6s200_driver/out/tool_pose",PoseStamped,self._pose_g2b_cb)


	def setup_param(self):
		# Setting up all the parameters here.
		self.matrix_c2g = np.array([[-0.9861,0.1651,0.0183,0.0489],
									[-0.1147,-0.5973,-0.7938,0.1440],
									[-0.1201,-0.7849,0.6079,-0.0366],
									[0.,0.,0.,1.0]])
		self.matrix_g2b = np.identity(4)

	def segmentation(self,pic):
		# Find strawberry as a target.
		# Return: pixels of strawberry
		return


	def get_position(self, pixels):
		# Get the position of strawberry using camera.
		# The position is relative to the camera.
		return
		

	def coordinate_transform(self,position_w2c):
		# Transform coordinate of strawberry.
		# From the pose relative to the camera
		# Step 1: pose relative to the hand -- Hand Eye Calibration.
		# Step 2: pose relative to the base -- Known by Kinova.
		position_w2g = np.matmul(self.matrix_c2g,position_w2c)
		# rospy.loginfo("position_w2g: %s", position_w2g)
		position_w2b = np.matmul(self.matrix_g2b,position_w2g)
		return position_w2b
		
	
	def _generate_tm(self,position,orientation):
		rm = Quaternion2Rotation(orientation)
		tm = []
		for i in xrange(3):
			tm.append(np.ndarray.tolist(np.append(rm[i],position[i])))
		tm = np.vstack((tm,[0.,0.,0.,1.]))
		return tm
	
	
	def _pose_g2b_cb(self,stampedpose):
		# rospy.loginfo("stamped Pose: %s", stampedpose)
		position = np.zeros(3)
		q = np.zeros(4)
		pose = stampedpose.pose
		position[0] = pose.position.x
		position[1] = pose.position.y
		position[2] = pose.position.z
		q[0] = pose.orientation.x
		q[1] = pose.orientation.y
		q[2] = pose.orientation.z
		q[3] = pose.orientation.w
		self.matrix_g2b = self._generate_tm(position,q)
	
	
	def get_pose(self):
		# generate pose to grasp strawberry.
		# This is irrelavant to the previous thing.
		# Probably be obtained by the picture.
		# At this stage, return the compensation: pose_whg
		return


	def joint_client(self,config):
		# Given a configuration, execute it.
		os.system("rosrun kinova_demo joints_action_client.py -v -r m1n6s200 degree -- %s %s %s %s %s %s" % config)
		return None


	def pose_client(self,pose,relative_state=False):
		# Given pose(position), execute with Mico.
		if relative_state == False:
			os.system("rosrun kinova_demo pose_action_client.py -v m1n6s200 mdeg -- %s %s %s %s %s %s" % (pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]))
		else:
			os.system("rosrun kinova_demo pose_action_client.py -v -r m1n6s200 mdeg -- %s %s %s %s %s %s" % (poes[0],pose[1],pose[2],pose[3],pose[4],pose[5]))
		# os.kill()
		return None


	def gripper_client(self,gripper_s):
		# Given gripper state signal, execute with gripper.
		rospy.loginfo("Enter Gripper Client")
		os.system("rosrun kinova_demo fingers_action_client.py m1n4s200 percent -- %s %s" % (gripper_s[0],gripper_s[1]))
		rospy.loginfo("Exit Gripper Client")
		return None

	def grasp(self):
		
		# read camera info.
		# segementation.
		# get position of strawberry (relevant to camera).
		
		position_w2c = np.array([[0.05],[0.14],[-0.04],[1]])
		# rospy.loginfo("position_w2c: %s",position_w2c)
		
		# coordinate transform.
		position = self.coordinate_transform(position_w2c)
		
		# rospy.loginfo("position: %s",position)
		# get pose of grasping.
		pose = np.append(position[0:3],np.array([45,45,45]))
		
		# rospy.loginfo("pose: %s",pose)
		# execute with pose.
		self.pose_client(pose)
		rospy.loginfo("Excute done.")
		# close the gripper to grasp.
		# rospy.sleep(4)
		# self.gripper_client([100,100])



def main():
	
	mico = Mico()
	
	rosthread = threading.Thread(name='ros_thread',target=rospy.spin)
	rosthread.start()
	
	rospy.sleep(0.5)
	# print "matrix_g2b", mico.matrix_g2b
	mico.grasp()
	
	
if __name__ == '__main__':
	try:
		print
		print "##### System Setup #####"
		print
		main()
	except rospy.ROSInterruptException:
		pass





















