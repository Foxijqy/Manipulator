#!/usr/bin/env python

# Sensor read in order to test softness.
# Qingyuan Jiang

import rospy
from std_msgs.msg import Bool, Float32

import matplotlib.pyplot as plt
import threading

import os


class Farmer():
	
	def __init__(self):

		self.setup_pam()
		pub = rospy.Publisher('gripper_state', Bool, queue_size=1)
		rospy.init_node('farmer_command',anonymous=True)
		self.rate = rospy.Rate(self.update_rate)
		force_sub = rospy.Subscriber('force_pub',Float32,self.force_cb)
		self.msg = Bool()
		self.state = False
		self.force_list = []
		self.rosthread = threading.Thread(name='ros_thread',target=rospy.spin)
	
	def setup_pam(self):
		self.update_rate = 20
		self.close_ang = 100
		self.open_ang = 50

	def commander(self):
		while not rospy.is_shutdown():
			if self.state == False:
				self.state = true
			else:
				self.state = False
			self.msg.data = self.state
			rospy.publish(msg)
			rate.sleep()

	def force_cb(self,msg):
		force = msg.data
		self.force_list.append(force)

	def display(self):
		plt.plot(self.force_list)
		plt.show()
	
	def display_org(self):
		self.rosthread.start()
		while not rospy.is_shutdown():
			plt.plot(self.force_list)
			plt.show()

	def sampling(self):
		self.rosthread.start()
		self.close() 		# To be accomplished
		self.open()			# To be accomplished
		self.display()
	
	'''
	def close(self):
		ang = self.open_ang
		while ang < self.close_ang:
			os.system("rosrun kinova_demo fingers_action_client.py m1n6s200 percent -- %s %s" % (ang,ang))
			ang = ang + 2
			self.rate.sleep()
	'''

	def close(self):
		os.system("rosrun kinova_demo fingers_action_client.py m1n6s200 percent -- %s %s" % (self.close_ang,self.close_ang))

	'''
	def open(self):
		ang = self.close_ang
		while ang > self.open_ang:
			os.system("rosrun kinova_demo fingers_action_client.py m1n6s200 percent -- %s %s" % (ang,ang))
			ang = ang - 10
			self.rate.sleep()
	'''
	
	def open(self):
		os.system("rosrun kinova_demo fingers_action_client.py m1n6s200 percent -- %s %s" % (self.open_ang,self.open_ang))



if __name__ == '__main__':
	try:
		farmer = Farmer()
		# farmer.display_org()
		farmer.sampling()
	except rospy.ROSInterruptException:
		pass

























