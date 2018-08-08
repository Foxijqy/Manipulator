#!/usr/bin python

import tensorflow as tf
import numpy as np

input_num = 6
output_num = 6
x_data = np.linspace(-1, 1, 300).reshape((-1, input_num))	# Transform to line vector.
print "x_data: ", x_data

noise = np.random.normal(0, 0.05, x_data.shape)
print "noise: ", noise
y_data = np.square(x_data) + 0.5 + noise
print "y_data: ", y_data

xs = tf.placeholder(tf.float32, [None, input_num])	# Sample number unknow, egen_num: 6
ys = tf.placeholder(tf.float32, [None, output_num])
print "xs: ", xs
print "ys: ", ys

neuro_layer_1 = 3
w1 = tf.Variable(tf.random_normal([input_num, neuro_layer_1]))
b1 = tf.Variable(tf.zeros([1, neuro_layer_1]) + 0.1)
l1 = tf.nn.relu(tf.matmul(xs, w1) + b1)
print "w1: ",w1
print "b1: ",b1
print "l1: ",l1

neuro_layer_2 = output_num
w2 = tf.Variable(tf.random_normal([neuro_layer_1, neuro_layer_2]))
b2 = tf.Variable(tf.zeros([1, neuro_layer_2]) + 0.1)
l2 = tf.matmul(l1, w2) + b2

# reduction_indices = [0] Add collumn.
# reduction_indices = [i] Add row.
loss = tf.reduce_mean(tf.reduce_sum(tf.square((ys - l2)), reduction_indices=[1]))

# Choose Gradient Descent Optimizer method.
train = tf.train.GradientDescentOptimizer(0.001).minimize(loss)
# train = tf.train.AdamOptimizer(1e-1).minimize(loss)

init = tf.global_variables_initializer()
sess = tf.Session()
sess.run(init)

'''
for i in range(100000):
    sess.run(train, feed_dict={xs: x_data, ys: y_data})
    if i % 1000 == 0:
        print(sess.run(loss, feed_dict={xs: x_data, ys: y_data}))
'''

