import tensorflow as tf
import numpy as np
import rospy
import sys, os
from .debug import DebugPrinter


class LiveClassifier(object):

    def __init__(self, load_path, debug_printer):
        """
        Constructor
        
        :param load_path: path (str) where data should be loaded from
        """
        if load_path[-1] != "/":
            load_path += "/"
        self.model_load_path = load_path + "model"
        self.load_path = load_path
        self._debug_printer = debug_printer

        self.input_shape = (40, 40, 3)

        self.X = tf.placeholder(tf.float32, shape=[None, self.input_shape[0], self.input_shape[1], self.input_shape[2]], name="X")
        self.Y = tf.placeholder(tf.float32, shape=[None, 1], name="Y")

        self.M, self.M_logits = self.model(self.X)

        self.initialize_network()

    def predict(self, batch):
        out = self.session.run(self.M, feed_dict={self.X: batch})
        # return confidences as is, i.e. is the same order as the input
        return out

    def initialize_network(self):
        self.session = tf.Session()
        self.init = tf.global_variables_initializer()
        self.session.run(self.init)
        self.saver = tf.train.Saver()
        rospy.loginfo()
        rospy.loginfo("loading weights from '{}'...".format(self.model_load_path))
        self.saver.restore(self.session, self.model_load_path)
        rospy.loginfo("loaded successfully.")

    def leaky_relu(self, x, leak=0.2, name=''):
        return tf.maximum(x, x * leak, name=name)

    def model(self, X):

        init_op = tf.contrib.layers.xavier_initializer(uniform=True,
                                                       dtype=tf.float32)

        with tf.variable_scope("m", initializer=init_op, dtype=tf.float32):

            with tf.variable_scope("transform1"):
                out = tf.layers.conv2d(X, 9, [1, 1], strides=[1, 1],
                                       padding="same")
                out = tf.layers.batch_normalization(out)
                out = self.leaky_relu(out, name="leaky")

            with tf.variable_scope("transform2"):
                out = tf.layers.conv2d(out, 3, [1, 1], strides=[1, 1],
                                       padding="same")
                out = tf.layers.batch_normalization(out)
                out = self.leaky_relu(out, name="leaky")

            with tf.variable_scope("conv1"):
                out = tf.layers.conv2d(out, 128, [3, 3], strides=[1, 1],
                                       padding="same")
                out = self.leaky_relu(out, name="leaky")

            with tf.variable_scope("conv2"):
                out = tf.layers.conv2d(out, 256, [3, 3], strides=[2, 2],
                                       padding="same")
                out = self.leaky_relu(out, name="leaky")

            with tf.variable_scope("dense"):
                out = tf.reshape(out, [-1, 20 * 20 * 256])
                out = tf.layers.dense(out, 100, activation=None)
                out = self.leaky_relu(out, name="leaky")

            with tf.variable_scope("output"):
                out = tf.layers.dense(out, 1, activation=None)
                logits = out
                output = tf.sigmoid(out)

        return output, logits
