from __future__ import absolute_import

import math
import rospy
import rospkg
import tf2_ros as tf2
from tf.transformations import euler_from_quaternion
import numpy as np

class VisualCompassFilter:
    def __init__(self):

        # TODO config file
        self.base_frame = 'base_footprint'
        self.camera_frame = 'camera_optical_frame'
        self.odom_frame = "odom"

        # Tf buffer time
        self.buffer_time = 10

        # Tf stuff
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(self.buffer_time))
        self.listener = tf2.TransformListener(self.tf_buffer)

        # Filter constant
        self.K = 0.99

        # Threshold to ignore values
        self.threshold = 0.3

        # Vektor which is got filtered by these filter
        self.filteredVector = None

        # Timestamp from this tick
        currentTimestamp = None

        # Timestamp from the last tick
        lastTimestamp = None

    def _getYawOrientationInBasefootprintFrame(self, visualCompassAngle, stamp):
        # Tf from camera frame to base frame
        camera_yaw_angle = 0
        correctedAngle = (visualCompassAngle) % (2 * math.pi)
        return correctedAngle

    def _initFilter(self, vector):
        rospy.loginfo("Init filter")
        # Set start value for the filter
        self.filteredVector = vector

    def filterMeasurement(self, angle, confidence, stamp):
        # Count too low confidences as zeros
        limitedConfidence = self._cutAtThreshold(confidence)

        # Tf messurement to base_footprint
        relativeYawAngle = self._getYawOrientationInBasefootprintFrame(angle, stamp)
        relativeYawVector = self._angleToVector(relativeYawAngle, limitedConfidence)

        # Set current timestamp
        self.currentTimestamp = stamp

        # Check if its the first iteration, if it is, init the filter
        if self.filteredVector is None:
            self._initFilter(relativeYawVector)
        else:
            # Correct old vector with odom data
            filteredVectorWithOdomOffset = self._odomOffset(self.filteredVector, self.lastTimestamp, self.currentTimestamp)
            # Run filter
            self.filteredVector = self._runFilter(filteredVectorWithOdomOffset, relativeYawVector)

        # Set previous timestamp
        self.lastTimestamp = self.currentTimestamp
        # Return vector as angle and confidence
        return self._vectorToAngle(self.filteredVector)

    def _cutAtThreshold(self, confidence):
        # Set everythin under the threshold to zero.
        if confidence <= self.threshold:
            confidence = 0
        return confidence

    def _odomOffset(self, vector, lastTimestamp, currentTimestamp):
        # Check if capture is not too long ago for the tf buffer
        if rospy.Duration(self.buffer_time) <= rospy.Time.now() - lastTimestamp:
            rospy.logwarn("No new values from visual compass for {} seconds".format(self.buffer_time))
            return np.array([0,0], dtype=np.float32)
        # Get odom difference
        oldOrientation = self._getYawFromTf(self.odom_frame, self.base_frame, lastTimestamp)
        currentOrientation = self._getYawFromTf(self.odom_frame, self.base_frame, currentTimestamp)
        odomOffsetYawAngle = (currentOrientation - oldOrientation)

        # Correct by odom difference
        odomCorrectedVector = self._rotateVectorByAngle(vector, odomOffsetYawAngle)
        return odomCorrectedVector
    
    def _angleToVector(self, angle, confidence):
        # Generate a vector from an angle and an confidence input
        return np.array([math.sin(angle) * confidence,
                         math.cos(angle) * confidence], dtype=np.float32)

    def _vectorToAngle(self, vector):
        # Converts an vector to an angle and an confidence value
        confidence = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        angle = (math.atan2(vector[0], vector[1])) % (math.pi * 2)
        angle = angle % (math.pi * 2)
        return angle, confidence

    def _rotateVectorByAngle(self, vector, offsetAngle):
        angle, confidence = self._vectorToAngle(vector)
        angle = (angle + offsetAngle) % (math.pi * 2)
        return self._angleToVector(angle, confidence)

    def _runFilter(self, oldVectorWithOdom, currentVector):
        # Magic
        new_vector_confidence = self._vectorToAngle(currentVector)[1]
        dynamicK = self.K * (1 - (new_vector_confidence * 0.5))
        return oldVectorWithOdom * dynamicK + currentVector * (1 - dynamicK)

    def _getYawFromTf(self, frame1, frame2, stamp, timeout=0.5):
        # Make a tf and get the yaw angle
        orientation = self.tf_buffer.lookup_transform(frame1, frame2, stamp, timeout=rospy.Duration(timeout)).transform.rotation
        yaw_angle = (euler_from_quaternion((orientation.x, 
                                            orientation.y, 
                                            orientation.z, 
                                            orientation.w))[2]) % (2 * math.pi)
        return yaw_angle
    
    def getFilteredValue(self):
        # Returns the filter value corrected by the newest odom offset (in development)
        now = rospy.Time.now()
        vector = self._odomOffset(self.filteredVector, self.cu<wrrentTimestamp, now)
        return self._vectorToAngle(vector)
