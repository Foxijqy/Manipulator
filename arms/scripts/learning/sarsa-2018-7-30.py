#!/usr/bin/env python

# Attempt for Sarsa learning using V-REP scene.
# Qingyuan Jiang

import numpy as np
import rospy
import math
from std_msgs.msg import Float64, Bool, Float64MultiArray
import threading

class Agent():
	def __init__(self,config_init):
		self.setup_pam()
		self.theta = np.zeros(6)
		self.action_space = np.array([[1,0,0,0,0,0],[-1,0,0,0,0,0],[0,1,0,0,0,0],[0,-1,0,0,0,0],[0,0,1,0,0,0],[0,0,-1,0,0,0],[0,0,0,1,0,0],[0,0,0,-1,0,0],[0,0,0,0,1,0],[0,0,0,0,-1,0],[0,0,0,0,0,1],[0,0,0,0,0,-1]])

		# Initialize ros message.
		self.node = rospy.init_node('Trajector', anonymous=False)
		self.act_pub = rospy.Publisher('joint_controller/command',Float64MultiArray,queue_size = 1)
		rospy.Subscriber("action_feedback",Float64MultiArray,self.act_callback)
		self.reward = 0
		self.deter = False
		self.reset(config_init)
		self.listening = True

	def setup_pam(self):
		self.ALPHA = 0.9
		self.GAMMA = 0.9
		self.EPSILON = 0.5
		self.TIME_DELAY = 0.5
		self.STEP = 10
		self.joint_limit = [[0,360],[47,313],[19,341],[0,360],[0,360],[0,360]]

	def reset(self,config_init):
		# Reset through ros
		# Publish config
		self.state = config_init
		self.execute(config_init)

	def gen_opt_act(self):
		act_index = np.argmax(self.theta*self.action_space)
		act = np.zeros(6)
		act[act_index/2] = np.power(-1,act_index)*self.STEP
		return act

	def find_act(self):
		if np.random.random() < self.EPSILON:
			act_index = np.random.random_integers(0,11)
			act = np.zeros(6)
			act[act_index/2] = np.power(-1,act_index)*self.STEP
			return act
		else:
			return self.gen_opt_act()

	def execute(self,config):
		# execute configure and send back states and reward
		rospy.loginfo("Exec: %s", config)
		config_p = []
		for i in xrange(len(config)):
			config_p.append(config[i]*math.pi/180)
		array_pub = Float64MultiArray(data=config_p)
		self.act_pub.publish(array_pub)

	def run(self,action):
		config = self.state + action
		# self.state = self.state + action
		# self.regulate(self.state)
		self.regulate(config)
		self.execute(config)
		rospy.sleep(self.TIME_DELAY)
		return self.reward, self.state, self.deter
	
	def act_callback(self,array):
		info = array.data
		rospy.loginfo("Action callback: %s", info[0:2])
		self.reward = info[0]
		self.deter = info[1]
		for i in xrange(6):
			self.state[i] = info[i+2]*180/math.pi
		rospy.loginfo("State refresh: %s", self.state)

	def regulate(self,config):
		config_r = config
		for i in xrange(6):
			if config_r[i] < self.joint_limit[i][0]:
				config_r[i] = self.joint_limit[i][0]
			elif config_r[i] > self.joint_limit[i][1]:
				config_r[i] = self.joint_limit[i][1]
		return config_r


def dot(theta,config,act):
	return np.dot(theta,config) + np.dot(theta,act)

class StoppableThread(threading.Thread):	# Not using at this stage.
	def __init__(self):
		super(StoppableThread, self).__init__()
		self._stop_event = threading.Event()
	
	def stop(self):
		self._stop_event.set()
	
	def stopped(self):	
		return self._stop_event.is_set()

''' Train the Agent '''

def main():

	total_episodes = 50

	i = 0
	total_reward = []
	total_length = []

	config_init = [180,180,180,180,180,180]
	agent = Agent(config_init)

	act = agent.find_act()
	s = agent.state

	rosthread = threading.Thread(name='ros_thread',target=rospy.spin)
	rosthread.start()

	while i < total_episodes:

		print
		rospy.loginfo ("##### Loop : %s ###############",i)

		r, s_new, d = agent.run(act)
		act_n = agent.find_act()
		agent.theta = agent.theta + agent.ALPHA*(r + agent.GAMMA*dot(agent.theta,s_new,act_n) - dot(agent.theta,s,act))*agent.theta

		rospy.loginfo("theta: %s",agent.theta)

		agent.state = s_new
		rospy.loginfo("state: %s",agent.state)
		act = act_n
		rospy.loginfo("act: %s", act)

		if d == True:
		    break
		else:
			i = i + 1

	# rosthread.release()
	print "### function end ###"
	#agent.listening = False
	#rospy.on_shutdown(hook)
	# agent.listening = False

if __name__ == '__main__':
	try:
		print "q-learning demo"
		main()
	except rospy.ROSInterruptException:
		pass





