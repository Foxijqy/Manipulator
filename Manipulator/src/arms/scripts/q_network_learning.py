#!/usr/bin/env python

# Attempt for Sarsa learning using V-REP scene.
# Qingyuan Jiang

import numpy as np
import rospy
import math
from std_msgs.msg import Float32, Bool, Float32MultiArray
import threading

import tensorflow as tf

class Agent():

	def __init__(self,config_init):

		self.setup_pam()
		self.action_space = np.array([
			[1,0,0,0,0,0],[-1,0,0,0,0,0],
			[0,1,0,0,0,0],[0,-1,0,0,0,0],
			[0,0,1,0,0,0],[0,0,-1,0,0,0],
			[0,0,0,1,0,0],[0,0,0,-1,0,0],
			[0,0,0,0,1,0],[0,0,0,0,-1,0],
			[0,0,0,0,0,1],[0,0,0,0,0,-1]
		])

		# Initialize ros message.
		self.node = rospy.init_node('Trajector', anonymous=False)
		self.act_pub = rospy.Publisher('joint_controller/command',Float32MultiArray,queue_size = 1)
		rospy.Subscriber("action_feedback",Float32MultiArray,self.act_callback)
		self.r_reward = 0
		self.r_deter = False

		self.reset(config_init)

		self.create_network()
		self.sess = tf.InteractiveSession()
		init = tf.global_variables_initializer()
		self.sess.run(init)
		
	def setup_pam(self):
		self.ALPHA = 0.9
		self.GAMMA = 0.9
		self.EPSILON = 0.5
		self.TIME_DELAY = 0.5
		self.STEP = 10
		self.learning_rate = 0.1
		self.joint_limit = [[0,360],
							[47,313],
							[19,341],
							[0,360],
							[0,360],
							[0,360]]

	def reset(self,config_init):
		# Reset through ros
		# Publish config
		self.state = np.array(config_init)
		self.r_state = np.array(config_init)
		self.execute(config_init)

	def create_network(self):
		self.q_input = tf.placeholder(shape=[1,6],dtype=tf.float32)
		self.w = tf.Variable(tf.random_uniform([6,12],0,0.01))
		self.q_out = tf.matmul(self.q_input,self.w)
		self.predict = tf.argmax(self.q_out, 1)

		self.nextQ = tf.placeholder(shape=[1,12],dtype=tf.float32)
		self.loss = tf.reduce_sum(tf.square(self.nextQ-self.q_out))
		self.trainer = tf.train.GradientDescentOptimizer(self.learning_rate)
		self.updateModel = self.trainer.minimize(self.loss)

	def get_q_list(self,state):
		return self.sess.run(self.q_out,feed_dict={self.q_input:state})

	def find_opt_act(self,state):	# Action are represented as index.
		opt_act = self.sess.run(self.predict,feed_dict={self.q_input:state})
		return opt_act

	def find_act(self,state):	# Contrib: reduce epsilon.
		act = self.find_opt_act(state)
		if np.random.random() < self.EPSILON:
			r_act = np.random.random_integers(0,11)
			act[0] = r_act
		return act

	def execute(self,config):
		# execute configure and send back states and reward
		rospy.loginfo("Exec: %s", config)
		config_p = []
		for i in xrange(len(config)):
			config_p.append(config[i]*math.pi/180)
		array_pub = Float32MultiArray(data=config_p)
		self.act_pub.publish(array_pub)

	def run(self,act):
		action = self.action_space[act]*self.STEP
		config = self.state + action
		# self.state = self.state + action
		# self.regulate(self.state)
		config_r = self.regulate(config)
		self.execute(config_r)
		rospy.sleep(self.TIME_DELAY)
		return self.r_state, self.r_reward, self.r_deter
	
	def act_callback(self,array):
		info = array.data
		rospy.loginfo("Action callback: %s", info[0:2])
		self.r_reward = info[0]
		self.r_deter = info[1]
		for i in xrange(6):
			self.r_state[0,i] = info[i+2]*180/math.pi
		self.r_state = np.array(self.r_state)
		rospy.loginfo("State refresh: %s", self.r_state)

	def regulate(self,config):
		config_r = config[0]
		for i in xrange(6):
			if config_r[i] < self.joint_limit[i][0]:
				config_r[i] = self.joint_limit[i][0]
			elif config_r[i] > self.joint_limit[i][1]:
				config_r[i] = self.joint_limit[i][1]
		return config_r

''' Train the Agent '''

def main():

	TOTAL_EPISODES = 10
	MAX_EPI = 99

	config_init = np.array([[180,180,180,180,180,180]])
	agent = Agent(config_init)

	rosthread = threading.Thread(name='ros_thread',target=rospy.spin)
	rosthread.start()

	for i in xrange(TOTAL_EPISODES):

		print
		rospy.loginfo ("##### Episode : %s #####",i)

		agent.reset(config_init)

		for j in xrange(MAX_EPI):

			s = agent.state
		
			q_list = agent.get_q_list(s)
			act = agent.find_act(s)
			s_new,r,d = agent.run(act)

			q_list_new = agent.get_q_list(s_new)
			maxQ = np.max(q_list_new)
			targetQ = q_list

			targetQ[0,act] = r + agent.GAMMA*maxQ

			# act_n = agent.find_act(s_new)
			# Train our network using target and predicted Q values.
			_,w1 = agent.sess.run([agent.updateModel,agent.w],
					feed_dict={agent.q_input:s,agent.nextQ:targetQ})

			agent.state = s_new

			rospy.loginfo("w1: %s",w1)
			rospy.loginfo("state: %s",agent.state)
			rospy.loginfo("act: %s", act)
			

			if d == True:
				break

		# for 'j' loop end.

	# for 'i' loop end.

	# rosthread.release()


if __name__ == '__main__':
	try:
		print "q-network learning demo"
		main()
	except rospy.ROSInterruptException:
		pass





