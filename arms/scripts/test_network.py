#!/usr/bin/env python


import tensorflow as tf
import numpy as np
from math import pi

learning_rate = 0.01
state_num = 6
action_num = 12

state_input = tf.placeholder(shape=[1,state_num],dtype=tf.float32)
q_target = tf.placeholder(shape=[1,action_num],dtype=tf.float32)


neuro_layer_1 = 12
w1 = tf.Variable(tf.random_uniform([state_num,neuro_layer_1],0,0.01))
b1 = tf.Variable(tf.zeros([1,neuro_layer_1]) + 0.1)
l1 = tf.nn.relu(tf.matmul(state_input,w1) + b1)


w2 = tf.Variable(tf.random_uniform([neuro_layer_1,action_num],0,0.01))
b2 = tf.Variable(tf.zeros([1,action_num]) + 0.1)
q_out = tf.matmul(l1,w2) + b2

# predict = tf.argmax(q_out, 1)

loss = tf.reduce_sum(tf.square(q_target - q_out))
trainer = tf.train.GradientDescentOptimizer(learning_rate)
updateModel = trainer.minimize(loss)


sess = tf.Session()
init = tf.global_variables_initializer()
sess.run(init)


#state = np.array([[180.,180.,180.,180.,180.,180.]])
state = np.array([[pi,pi,pi,pi,pi,pi]])
q_list = np.array([[2.,2.,2.,2.,2.,2.,2.,2.,2.,2.,2.,2.]])









for i in xrange(20):
	_,los = sess.run([updateModel,loss], feed_dict={q_target:q_list,state_input:state})
	print los













