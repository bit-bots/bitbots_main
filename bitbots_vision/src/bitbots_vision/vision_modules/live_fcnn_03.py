import os
import tensorflow as tf
import rospy


class FCNN03:
    """
    Defines the FCNN neural network
    """
    def __init__(self, load_path):
        rospy.logdebug("Setting up ball detection: FCNN03")
        # Make fcnn load path
        self._load_path = os.path.join(load_path, "model_final")
        # Define input and output shape of the fcnn
        self.input_shape = (150, 200, 3)  # y, x, z
        self.output_shape = (150, 200, 1)  # y, x, z

        # Define tensorflow placeholders
        with tf.variable_scope("placeholders"):
            self._keep_prob = tf.placeholder("float", name="keep_prob")
            self.X = tf.placeholder(
                tf.float32,
                shape=[
                    None,
                    self.input_shape[0],
                    self.input_shape[1],
                    self.input_shape[2]],
                name="X")
            self.Y = tf.placeholder(
                tf.float32,
                shape=[
                    None,
                    self.output_shape[0],
                    self.output_shape[1],
                    self.output_shape[2]],
                name="Y")

        # Create network
        self._fcnn_out, self._fcnn_logits = self._fcnn_model()

        # Init network & load weights
        self._initialize_network()

    def predict(self, batch):
        """
        Runs the fcnn neural network
        """
        return self.session.run(self._fcnn_out, feed_dict={self.X: batch, self._keep_prob: 1.0})

    def _initialize_network(self):
        """
        Init tensorflow
        """
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.session = tf.Session(config=config)
        self.init = tf.global_variables_initializer()
        self.session.run(self.init)
        self.saver = tf.train.Saver()
        rospy.logdebug("loading weights from '{}'...".format(self._load_path))
        self.saver.restore(self.session, self._load_path)
        rospy.logdebug("loaded successfully.")

    def _fcnn_model(self):
        """
        Defines the fcnn model layers
        """
        with tf.variable_scope("conv", dtype=tf.float32):
            #################
            # Encoding part #
            #################
            with tf.variable_scope("conv1"):
                # 150x200x3
                out = tf.layers.conv2d(self.X, 16, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)
                before_maxpool1 = out
                out = tf.layers.max_pooling2d(out, [2, 2], strides=[2, 2], padding="same")
                maxpool1 = out

            with tf.variable_scope("conv2"):
                # 75x100x8
                out = tf.layers.conv2d(out, 32, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("conv3"):
                # 75x100x16
                out = tf.layers.conv2d(out, 32, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("concat1"):
                # 75x100x16
                out = tf.concat([out, maxpool1], 3)
                concat1 = out
                # 75x100x(16+8)

            with tf.variable_scope("conv4"):
                out = tf.layers.max_pooling2d(out, [2, 2], strides=[2, 2], padding="same")
                # 38x50x(16+8)
                maxpool2 = out
                out = tf.layers.conv2d(out, 64, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("conv5"):
                # 38x50x32
                out = tf.layers.conv2d(out, 64, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("concat2"):
                out = tf.concat([out, maxpool2], 3)
                concat2 = out
                # 38x50x(32+24)

            with tf.variable_scope("conv6"):
                # 38x50x(32+24)
                out = tf.layers.conv2d(out, 128, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)
                # 38x50x64

            with tf.variable_scope("conv7"):
                # 38x50x64
                out = tf.layers.conv2d(out, 128, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)
                # 38x50x64

            #################
            # Decoding part #
            #################
            with tf.variable_scope("concat4"):
                # 38x50x64
                out = tf.image.resize_images(out, [75, 100], tf.image.ResizeMethod.BILINEAR)
                # 75x100x64
                out = tf.concat([out, concat1], 3)
                # 75x100x(64+64)

            with tf.variable_scope("conv13"):
                # 75x100x(64+64)
                out = tf.layers.conv2d(out, 64, [1, 1], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("conv14"):
                # 75x100x64
                out = tf.layers.conv2d(out, 32, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("conv15"):
                # 75x100x32
                out = tf.layers.conv2d(out, 32, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("concat5"):
                # 75x100x32
                out = tf.image.resize_images(out, [150, 200], tf.image.ResizeMethod.BILINEAR)
                # 150x200x32
                out = tf.concat([out, before_maxpool1], 3)
                # 150x200x(32+32)

            with tf.variable_scope("conv16"):
                # 150x200x(32+32)
                out = tf.layers.conv2d(out, 16, [1, 1], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("conv17"):
                # 150x200x16
                out = tf.layers.conv2d(out, 16, [3, 3], strides=[1, 1], padding="same")
                out = tf.layers.batch_normalization(out)
                out = tf.nn.relu(out)
                out = tf.nn.dropout(out, keep_prob=self._keep_prob)

            with tf.variable_scope("conv18"):
                # 150x200x16
                out = tf.layers.conv2d(out, 1, [3, 3], strides=[1, 1], padding="same")
                # out = tf.layers.batch_normalization(out)
                logits = out
                out = tf.maximum(tf.minimum(out, 1.0), 0.0)
                # 150x200x1

        return out, logits


