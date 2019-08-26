#!/usr/bin/env python

# Attempt for Q-Network learning using V-REP scene.
# Qingyuan Jiang

import numpy as np
import rospy
from math import pi
from std_msgs.msg import Float32, Bool, Float32MultiArray
import threading
import os
import tensorflow as tf
import matplotlib.pyplot as plt
import random
from collections import deque


''' Create an Agent '''

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

		self.reset(config_init)

		self.create_network()
		self.sess = tf.Session()
		init = tf.global_variables_initializer()
		self.sess.run(init)
		
		self.replay_memory_store = deque()
		self.memory_counter = 0
		

	def setup_pam(self):
		self.ALPHA = 0.9
		self.GAMMA = 0.9
		self.EPSILON = 0.4
		self.TIME_DELAY = 0.1
		self.STEP = 0.1
		self.learning_rate = 0.01
		self.joint_limit = [[0,360],
							[47,313],
							[19,341],
							[0,360],
							[0,360],
							[0,360]]
		self.BATCH = 20
		self.memory_size = 5000
		self.ACT_THRESH = -20


	def reset(self,config_init):
		# Reset through ros, publish config
		# rospy.loginfo("##### Reset %s #####",config_init)
		self.state = np.array([[0.,0.,0.,0.,0.,0.]])		# State is actually "feature": Related pose to the target.
		self.s_config = config_init
		
		self.r_state = np.array([[0.,0.,0.,0.,0.,0.]])	# Receiving state date from environment.
		self.r_reward = 0
		self.r_done = False
		self.execute(config_init)
		rospy.sleep(self.TIME_DELAY)
		self.state = self.r_state
		#rospy.loginfo("return run function: %s",(self.r_reward,self.r_done))
		

	def create_network(self):
	
		self.state_num = 6
		self.action_num = 12
		
		self.state_input = tf.placeholder(shape=[None,self.state_num],dtype=tf.float32)
		self.q_target = tf.placeholder(shape=[None,self.action_num],dtype=tf.float32)
		
		nuero_layer_1 = 12
		self.w1 = tf.Variable(tf.random_uniform([self.state_num,nuero_layer_1],0,0.01))
		self.b1 = tf.Variable(tf.zeros([1,nuero_layer_1]) + 0.1)
		self.l1 = tf.nn.relu(tf.matmul(self.state_input,self.w1) + self.b1)
		
		self.w2 = tf.Variable(tf.random_uniform([nuero_layer_1,self.action_num],0,0.01))
		self.b2 = tf.Variable(tf.zeros([1,self.action_num]) + 0.1)
		self.q_out = tf.matmul(self.l1,self.w2) + self.b2
		
		self.predict = tf.argmax(self.q_out, 1)

		self.loss = tf.reduce_mean(tf.square(self.q_target-self.q_out))
		self.trainer = tf.train.GradientDescentOptimizer(self.learning_rate)
		self.updateModel = self.trainer.minimize(self.loss)		


	def get_q_list(self,state):
		return self.sess.run(self.q_out,feed_dict={self.state_input:state})


	def find_opt_act(self,state):	# Action are represented as index.
		opt_act = self.sess.run(self.predict,feed_dict={self.state_input:state})
		return opt_act


	def find_act(self,state):	# Contrib: reduce epsilon.
		act = self.find_opt_act(state)
		if np.random.random() < self.EPSILON:
			r_act = np.random.random_integers(0,11)
			act[0] = r_act
		return act


	def execute(self,config):
		# rospy.loginfo("Execute: %s",config)
		array_pub = Float32MultiArray(data=config)
		self.act_pub.publish(array_pub)


	def run(self,act):
		action = self.action_space[act]*self.STEP
		config = self.s_config + action
		config_r = self.regulate(config)
		self.execute(config_r)
		rospy.sleep(self.TIME_DELAY)
		#rospy.loginfo("return run function: %s",(self.r_reward,self.r_done))
		return self.r_state, self.r_reward, self.r_done
	
	
	def act_callback(self,array):
		info = array.data
		#rospy.loginfo("Action callback: %s", info[0:2])
		#rospy.loginfo("Action callback: %s", info)
		self.r_reward = info[0]
		
		if info[1] == 1:
			self.r_done = True
		elif info[1] == 0:
			self.r_done = False
		else:
			raise Exception("Passed 'done' info should be 1 or 0.")
		
		# Update inner configure record.(In case of self-collision)
		rospy.loginfo("old_config: %s",self.s_config)
		rospy.loginfo("config: %s",info[2:8])
		
		s_config_old = self.s_config
		for i in xrange(6):
			self.s_config[i] = info[i+2]
			
		rospy.loginfo("s_config: %s",s_config)
		
		if self.is_equal(s_config_old,self.s_config):
			rospy.loginfo("##### SEFL COLLISION OR REACH JOINT LIMIT #####")
			self.r_reward = self.ACT_THRESH
		# Receive state here.
		for j in xrange(6):
			self.r_state[0,j] = info[j+8]
		
		#rospy.loginfo("State refresh: %s", self.r_state)


	def is_equal(self,config1,config2):
		if len(config1) != len(config2):
			raise Exception("the length of two configuration should be equal")
		for i in xrange(len(config1)):
			if config1[i] != config2[i]:
				return False
		return True
		
		
	def regulate(self,config):
		config_r = config[0]
		for i in xrange(6):
			if config_r[i] < self.joint_limit[i][0]*pi/180.:
				config_r[i] = self.joint_limit[i][0]*pi/180.
			elif config_r[i] > self.joint_limit[i][1]*pi/180.:
				config_r[i] = self.joint_limit[i][1]*pi/180.
		return config_r


	def save_store(self, current_state, current_act, current_reward, next_state, done):
		# current_action = self.action_space[current_act]*self.STEP
		
		self.replay_memory_store.append((
			current_state,
			current_act,
			current_reward,
			next_state,
			done))
			
		if len(self.replay_memory_store) > self.memory_size:
			self.replay_memory_store.popleft()
		
		self.memory_counter = self.memory_counter + 1
		
	
	def experience_replay(self):
	
		# print self.replay_memory_store
		
		if self.memory_counter > self.BATCH:
			batch = self.BATCH
		else:
			batch = self.memory_counter

		minibatch = random.sample(self.replay_memory_store, batch)
		
		batch_state = None
		batch_action = None
		batch_reward = None
		batch_next_state = None
		batch_done = None
		
		for index in xrange(len(minibatch)):
			if batch_state is None:
				batch_state = minibatch[index][0]
			else:
				batch_state = np.vstack((batch_state,minibatch[index][0]))
				
			if batch_action is None:
				batch_action = minibatch[index][1]
			else:
				batch_action = np.vstack((batch_action,minibatch[index][1]))
				
			if batch_reward is None:
				batch_reward = minibatch[index][2]
			else:
				batch_reward = np.vstack((batch_reward,minibatch[index][2]))
				
			if batch_next_state is None:
				batch_next_state = minibatch[index][3]
			else:
				batch_next_state = np.vstack((batch_next_state,minibatch[index][3]))
				
			if batch_done is None:
				batch_done = minibatch[index][4]
			else:
				batch_done = np.vstack((batch_done,minibatch[index][4]))
		
		q_list = self.sess.run(self.q_out,feed_dict={self.state_input:batch_state})
		q_next = self.sess.run(self.q_out, feed_dict={self.state_input:batch_next_state})
		
		q_target = q_list
		# print q_target
		
		for i in xrange(len(minibatch)):
			
			current_reward = batch_reward[i][0]
			current_action = batch_action[i][0]
			
			if current_reward <= self.ACT_THRESH:
				q_target[i,current_action] = current_reward
			else:
				q_target[i,current_action] = current_reward + self.GAMMA * np.max(q_next[i])
			
		_,cost = self.sess.run([self.updateModel,self.loss],feed_dict={self.state_input:batch_state,self.q_target:q_target})
		
		# Return something is necessary.
		return cost
		
		

''' Train the Agent '''

def main():

	TOTAL_EPISODES = 20
	MAX_EPI = 500

	config_init = np.array([pi,pi,pi,pi,pi,pi])
	agent = Agent(config_init)

	rosthread = threading.Thread(name='ros_thread',target=rospy.spin)
	rosthread.start()

	rList = []
	cList = []
	
	step_index = 0
	OBSERVE = 200
	
	for i in xrange(TOTAL_EPISODES):

		#print
		config_init = np.array([pi,pi,pi,pi,pi,pi])
		agent.reset(config_init)
		rAll = 0
		cAll = 0
		
		for j in xrange(MAX_EPI):

			# print 
			s = agent.state
			act = agent.find_act(s)
			s_new,r,d = agent.run(act)

			agent.save_store(s,act,r,s_new,d)
			
			if step_index > OBSERVE:
				cost = agent.experience_replay()
				rospy.loginfo("cost: %s", cost)
				cAll = cAll + cost

			agent.state = s_new
			
			# For supervision.
			rAll = rAll + r
			
			if d == True:
				break
				
			step_index = step_index + 1
			
		# for 'j' loop end.
		
		rospy.loginfo("##### Episode %s Reward: %s #####" % (i,rAll))
		rList.append(rAll)
		cList.append(cAll)
		
		agent.EPSILON = agent.EPSILON - agent.EPSILON / TOTAL_EPISODES
		
	# for 'i' loop end.
	
	print
	print "##### Reward List #####", rList
	print
	print "##### Cost List #####", cList
	
	plt.plot(rList)
	plt.plot(cList)
	plt.show()






def network_test():

	init_config = np.array([pi,pi,pi,pi,pi,pi])
	agent = Agent(init_config)

	q_input = np.array([[0.3,0.2,0.4,0.2,0.1,0.5],[0.3,0.3,0.5,0.6,0.03,0.07]])
	q_list = np.array([[2.,2.,2.,2.,2.,2.,2.,2.,2.,2.,2.,2.],[3.,2.,1.,1.,1.,1.,3.,2.,1.,1.,1.,1.]])	

	# print "agent.w: ", agent.sess.run(agent.w)

	costList = []
	
	for i in xrange(100):

		_,los = agent.sess.run([agent.updateModel,agent.loss],
		feed_dict={agent.state_input:q_input,agent.q_target:q_list})
		# print "loss: ",los
		costList.append(los)
	
	print costList
	plt.plot(costList)
	plt.show()



def experience_replay_test():
	
	init_config = np.array([pi,pi,pi,pi,pi,pi])
	ag = Agent(init_config)
	
	# Add some experience.
	for i in xrange(1000):
		s = np.random.rand(6)
		#s = np.array([2.,2.,2.,2.,2.,2.])
		act = np.random.random_integers(0,11)
		reward = np.random.rand()
		#s_new = np.random.rand(6)
		s_new = np.array([0.2,0.3,0.4,0.4,0.3,0.2])
		d = False
		ag.save_store(s,act,reward,s_new,d)
	
	lossList = []
	for j in xrange(40):
		cost = ag.experience_replay()
		lossList.append(cost)
		
	plt.plot(lossList)
	plt.show()





if __name__ == '__main__':
	try:
		print 
		print "##################################################"
		print "########## Q-Network Learning Demo ###############"
		print "##################################################"
		print
		
		main()
		# network_test()
		# experience_replay_test()
	except rospy.ROSInterruptException:
		pass































