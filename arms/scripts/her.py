#!/usr/bin/env python

# Algorithm Implement of Hindsight Experience Replay (HER)
# 2018-8-26
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


DISTANCE_THRESH = 0.01
SUCCESS_REWARD = 10.
FAILED_REWARD = -1.0
ACT_THRESH = -20.

''' Create an Agent '''

class Agent():

	def __init__(self,config_init=np.array([pi,pi,pi,pi,pi,pi])):

		self.setup_pam()
		self.action_space = np.array([
		[1,0,0,0,0,0],[-1,0,0,0,0,0],
		[0,1,0,0,0,0],[0,-1,0,0,0,0],
		[0,0,1,0,0,0],[0,0,-1,0,0,0],
		[0,0,0,1,0,0],[0,0,0,-1,0,0],
		[0,0,0,0,1,0],[0,0,0,0,-1,0],
		[0,0,0,0,0,1],[0,0,0,0,0,-1]
		])
		self.action_num = len(self.action_space)
		self.state_num = self.config_dim + self.feat_num

		self.config_init = config_init
		
		# Initialize ros message.
		self.node = rospy.init_node('Trajector', anonymous=False)
		self.act_pub = rospy.Publisher('joint_controller/command',Float32MultiArray,queue_size = 1)
		rospy.Subscriber("action_feedback",Float32MultiArray,self.act_callback)

		self.reset()

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
		self.joint_limit = [[0.,360.],[47.,313.],[19.,341.],
							[0.,360.],[0.,360.],[0.,360.]]
		self.BATCH = 20
		self.memory_size = 5000
		self.ACT_THRESH = -20
		
		self.config_dim = 6
		self.feat_num = 6


	def reset(self):
		# Reset through ros, publish config
		self.s_config = self.config_init						# s_config: array([1,2,3,4,5,6])
		self.s_feat = np.zeros(self.feat_num)					# s_feat: array([1,2,3,4,5,6])
		self.state = self.get_state(self.s_config,self.s_feat)	# state: array([[1,2,3..12]])
		
		self.r_config = np.zeros(self.config_dim)
		self.r_feat = np.zeros(self.feat_num)
		self.r_state = self.get_state(self.r_config,self.r_feat)
		self.r_reward = 0
		self.r_done = False
		
		self.act_dict = {}
		self.action_locker_setup()
		
		self.execute(self.s_config)
		rospy.sleep(self.TIME_DELAY)
		self.set_state(self.r_state)
		# rospy.loginfo("State Check: %s", self.state)
		#rospy.loginfo("return run function: %s",(self.r_reward,self.r_done))
		
		self.target_pose = np.array([0.15,0.25,0.5,0.,0.,0.])
		self.target_tem_pose = np.array([0.15,0.25,0.5,0.,0.,0.])
		
		
	def action_locker_setup(self):
		for i in xrange(self.action_num):
			self.act_dict[i] = False
		
		
	def create_network(self):
		
		self.state_input = tf.placeholder(shape=[None,self.state_num],dtype=tf.float32)
		self.q_target = tf.placeholder(shape=[None,self.action_num],dtype=tf.float32)
		
		nuero_layer_1 = 12
		self.w1 = tf.Variable(tf.random_uniform([self.state_num,nuero_layer_1],0,0.01))
		self.b1 = tf.Variable(tf.zeros([1,nuero_layer_1]) + 0.1)
		self.l1 = tf.nn.relu(tf.matmul(self.state_input,self.w1) + self.b1)
		
		nuero_layer_2 = 12
		self.w2 = tf.Variable(tf.random_uniform([nuero_layer_1,nuero_layer_2],0,0.01))
		self.b2 = tf.Variable(tf.zeros([1,nuero_layer_2]) + 0.1)
		self.l2 = tf.nn.relu(tf.matmul(self.l1,self.w2) + self.b2)
		
		self.w3 = tf.Variable(tf.random_uniform([nuero_layer_2,self.action_num],0,0.01))
		self.b3 = tf.Variable(tf.zeros([1,self.action_num]) + 0.1)
		self.q_out = tf.matmul(self.l2,self.w3) + self.b3
		
		self.predict = tf.argmax(self.q_out, 1)

		self.loss = tf.reduce_sum(tf.square(self.q_target-self.q_out))
		self.trainer = tf.train.GradientDescentOptimizer(self.learning_rate)
		self.updateModel = self.trainer.minimize(self.loss)
		
		
	def get_feature(self,tip_pose):			# Could be revised later.
		return tip_pose - self.target_pose
		
		
	def get_state(self,config,feature):
		if len(config) != self.config_dim:
			raise Exception("Configuration length should be 6")
		elif len(feature) != self.feat_num:
			raise Exception("Relative pose length should be 6")
		else:
			return np.array([np.concatenate((config,feature))])
	
	
	def set_state(self,state):
		if len(state[0]) != self.config_dim + self.feat_num:
			raise Exception("State length error.")
		else:
			self.s_config = state[0][0:self.config_dim]
			self.s_feat = state[0][self.config_dim : self.config_dim + self.feat_num]
			self.state = state
			
	
	def get_q_list(self,state):
		return self.sess.run(self.q_out,feed_dict={self.state_input:state})


	def find_opt_act(self,state):	# Action are represented as index.
		# rospy.loginfo("state %s",state)
		opt_act,q_list = self.sess.run([self.predict,self.q_out],feed_dict={self.state_input:state})
		# rospy.loginfo("opt_act: %s",opt_act)
		# rospy.loginfo("q_list: %s", q_list)
		# rospy.loginfo("q_list max: %s",np.max(q_list))
		return opt_act


	def find_act(self,state):
		act = self.find_opt_act(state)					# act: [int]
		if np.random.random() < self.EPSILON:			# If random.
			r_act = np.random.random_integers(0,11)
			while self.act_dict[r_act] != False:
				# rospy.loginfo("############### Re-random act. ###############")
				r_act = np.random.random_integers(0,11)
			act[0] = r_act
			return act
		elif self.act_dict[act[0]] == True:				# Find another action.
			r_act = np.random.random_integers(0,11)
			while self.act_dict[r_act] != False:
				# rospy.loginfo("############### Re-random act. ###############")
				r_act = np.random.random_integers(0,11)
			act[0] = r_act
			return act
		else:
			return act


	def run(self,act):
		self.action_locker_setup()
		self.previous_act = act
		
		action = self.action_space[act[0]]*self.STEP
		config = self.s_config + action
		config_r = self.regulate(config)
		self.execute(config_r)
		rospy.sleep(self.TIME_DELAY)
		# rospy.loginfo("return run function: %s",(self.r_reward,self.r_done))	# Used for time-check
		return self.r_state, self.r_reward, self.r_done
		
		
	def execute(self,config):
		# rospy.loginfo("Execute: %s",config)
		array_pub = Float32MultiArray(data=config)
		self.act_pub.publish(array_pub)	
	
	
	def act_callback(self,array):
		
		info = np.array(array.data)
		if len(info) != 14:
			raise Exception("ROS data length error.")
		
		self.r_reward = info[0]
		if info[1] == 1:
			self.r_done = True
		elif info[1] == 0:
			self.r_done = False
		else:
			raise Exception("Passed 'done' info should be 1 or 0.")
			
		self.r_config = info[2:2+self.config_dim]
		tip_pose = info[2+self.config_dim:2+self.config_dim+self.feat_num]
		self.r_feat = self.get_feature(tip_pose)
		self.r_state = self.get_state(self.r_config,self.r_feat)
		
		# Avoid self-collision by modelling reward function.
		if np.array_equal(self.s_config,self.r_config):
			# rospy.loginfo("##### SEFL COLLISION OR REACH JOINT LIMIT #####")
			# rospy.loginfo("state config %s", self.s_config)
			# rospy.loginfo("receive config %s", self.r_config)
			self.r_reward = self.ACT_THRESH
			self.act_dict[self.previous_act[0]] = True
		# rospy.loginfo("Callback Function End.")								# Used for time-check
		
		
	def regulate(self,config):
		config_r = config.copy()
		for i in xrange(6):
			if config_r[i] < self.joint_limit[i][0]*pi/180.:
				config_r[i] = self.joint_limit[i][0]*pi/180.
			elif config_r[i] > self.joint_limit[i][1]*pi/180.:
				config_r[i] = self.joint_limit[i][1]*pi/180.
		return config_r


	def save_store(self, current_state, current_act, current_reward, next_state, done):
		
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
	
		# rospy.loginfo("##### Experience Replay #####")
		
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
		
		q_target = q_list.copy()
		
		for i in xrange(len(minibatch)):		
			current_reward = batch_reward[i][0]
			current_action = batch_action[i][0]
			if current_reward <= self.ACT_THRESH:
				q_target[i,current_action] = current_reward
			else:
				q_target[i,current_action] = current_reward + self.GAMMA * np.max(q_next[i])
		# rospy.loginfo("q_target: %s",q_target)
			
		_,q_list,cost = self.sess.run([self.updateModel,self.q_out,self.loss],feed_dict={self.state_input:batch_state,self.q_target:q_target})
		
		# Return something is necessary.
		return q_list, cost
		
		

''' ##### Train the Agent ##### '''

def get_reward(state,action,state_new,goal):
	distance = 0
	for i in xrange(3):
		distance = distance + pow(state_new[0][i+6],2)
	if distance <= DISTANCE_THRESH:
		return SUCCESS_REWARD
	else:
		if np.array_equal(state[0][0:6],state_new[0][0:6]):
			return ACT_THRESH
		else:
			return FAILED_REWARD


def get_new_state(state,target_pose,original_pose):
	state[0][6:12] = state[0][6:12] + original_pose - target_pose
	return state
	

def her():

	# Setup up parameters.
	TOTAL_EPISODES = 800
	MAX_EPI = 100
	N = 50
	OBSERVE = 400
	
	init_config = np.array([pi,pi,pi,pi,pi,pi])
	ag = Agent(init_config)
	
	rosthread = threading.Thread(name='ros_thread',target=rospy.spin)
	rosthread.start()
	
	cList = []
	rList = []
	
	for i in xrange(TOTAL_EPISODES):
		
		# rospy.loginfo("##### Episode: %s #####",i)
		ag.reset()
		# ag.sample_goal()
		rAll = 0
		cAll = 0
		
		s_buffer = []
		for j in xrange(MAX_EPI):
			# print
			s = ag.state
			# rospy.loginfo("state: %s",s)
			a = ag.find_act(s)
			s_n,r,d = ag.run(a)
			ag.set_state(s_n)
			
			s_buffer.append([s,a,s_n,d])

			if d == True:
				break
		# for 'j' loop end.
		
		g_tem = s_buffer[-1][-2][0][6:12] + ag.target_pose
		# rospy.loginfo("g_tem: %s",g_tem)
		
		for j in xrange(MAX_EPI):
			s,a,s_n,d = s_buffer[j]
			g = ag.target_pose
			r = get_reward(s,a,s_n,g)
			ag.save_store(s,a,r,s_n,d)		
			# rospy.loginfo("Reward: %s", r)
			
			# Get new goals
			s_prime = get_new_state(s,g_tem,ag.target_pose)
			s_n_prime = get_new_state(s_n,g_tem,ag.target_pose)
			r_prime = get_reward(s_prime,a,s_n_prime,g_tem)
			ag.save_store(s_prime,a,r_prime,s_n_prime,d)		
			# rospy.loginfo("New Reward: %s", r_prime)
			
			# For supervision
			rAll = rAll + r
		# for 'j' loop end.
		
		if ag.memory_counter > OBSERVE:
			for t in xrange(N):
				# rospy.loginfo("Start Experience Replay: Time %s", t)
				q_list,cost = ag.experience_replay()
				cAll = cAll + cost
		# for 't' loop end.
		
		rospy.loginfo("Episode %s Reward: %s Cost: %s" % (i,rAll,cAll))
		rList.append(rAll)
		cList.append(cAll)
		ag.EPSILON = ag.EPSILON - ag.EPSILON / TOTAL_EPISODES
		
	# for 'i' loop end.
	
	plt.plot(rList)
	plt.plot(cList)
	plt.show()


def network_test():

	init_config = np.array([pi,pi,pi,pi,pi,pi])
	agent = Agent(init_config)

	q_input = np.array([[0.3,0.2,0.4,0.2,0.1,0.5,0.3,0.2,0.4,0.2,0.1,0.5],
						[0.3,0.3,0.5,0.6,0.03,0.07,0.3,0.3,0.5,0.6,0.03,0.07]])
	q_list = np.array([[2.,2.,2.,2.,2.,2.,2.,2.,2.,2.,2.,2.],
					   [3.,2.,1.,1.,1.,1.,3.,2.,1.,1.,1.,1.]])	

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

	
if __name__ == '__main__':
	try:
		print 
		print "##################################################"
		print "########## Q-Network Learning Demo ###############"
		print "##################################################"
		print
		
		# main()
		her()
		# network_test()
		# experience_replay_test()
	except rospy.ROSInterruptException:
		pass






''' Reference on DQN '''
'''
def dqn():

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
		agent.reset()
		rAll = 0
		cAll = 0
		
		for j in xrange(MAX_EPI):

			print
			s = agent.state
			act = agent.find_act(s)
			s_new,r,d = agent.run(act)

			agent.save_store(s,act,r,s_new,d)
			
			if step_index > OBSERVE:
				q_list, cost = agent.experience_replay()
				cAll = cAll + cost

			agent.set_state(s_new)
			
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
'''


'''
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
'''

'''
def create_network(self):
		
		self.state_input = tf.placeholder(shape=[None,self.state_num],dtype=tf.float32)
		self.q_target = tf.placeholder(shape=[None,self.action_num],dtype=tf.float32)
		
		nuero_layer_1 = 12
		self.w1 = tf.Variable(tf.random_uniform([self.state_num,nuero_layer_1],0,0.01))
		self.b1 = tf.Variable(tf.zeros([1,nuero_layer_1]) + 0.1)
		self.l1 = tf.nn.relu(tf.matmul(self.state_input,self.w1) + self.b1)
		
		nuero_layer_2 = 12
		self.w2 = tf.Variable(tf.random_uniform([nuero_layer_1,nuero_layer_2],0,0.01))
		self.b2 = tf.Variable(tf.zeros([1,nuero_layer_2]) + 0.1)
		self.l2 = tf.nn.relu(tf.matmul(self.l1,self.w2) + self.b2)
		
		self.w3 = tf.Variable(tf.random_uniform([nuero_layer_2,self.action_num],0,0.01))
		self.b3 = tf.Variable(tf.zeros([1,self.action_num]) + 0.1)
		self.q_out = tf.matmul(self.l2,self.w3) + self.b3
		
		self.predict = tf.argmax(self.q_out, 1)

		self.loss = tf.reduce_sum(tf.square(self.q_target-self.q_out))
		self.trainer = tf.train.GradientDescentOptimizer(self.learning_rate)
		self.updateModel = self.trainer.minimize(self.loss)		
'''

















