#! /usr/bin/env python3
import rospy
import rospkg
import math
import cv2
import numpy as np
from humanoid_league_msgs.msg import PoseWithCertaintyStamped
from tf_conversions.transformations import euler_from_quaternion


class VisualCompassVisualization():
    # type: () -> None
    """
    A simple CV window that displays the current orientation messure by the visual compass.
    """
    def __init__(self):
        # type: () -> None
        """
        Inits the helper
        """
        # Init ROS package
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_visual_compass')

        rospy.init_node('bitbots_visual_compass_demo_viz')
        rospy.loginfo('Launching demo tool!')

        rospy.Subscriber(
                'visual_compass',
                PoseWithCertaintyStamped,
                self.compass_callback,
                tcp_nodelay=True)

        self.scale = 100

        rospy.spin()

    def compass_callback(self, msg):
        """
        Processes the date from the visual compass and draws the graph
        """
        canvas = np.zeros((300,300), dtype=np.uint8)

        angle = ((2 * math.pi - euler_from_quaternion(msg.pose.pose.pose.orientation)[2]) - 0.5 * math.pi)
        length = msg.confidence * self.scale

        vektor = (int(math.cos(angle)*length), int(math.sin(angle)*length))

        center = (canvas.shape[0]/2, canvas.shape[1]/2)
        point = (center[0] + vektor[0], center[1] + vektor[1])

        img	= cv2.arrowedLine(canvas, tuple(center), tuple(point), (255,255,255), thickness = 3, tipLength = 0.3)

        output_str = "{} Deg | {}%".format(int(math.degrees(msg.orientation)), int(msg.confidence * 100))
        text_margin = 10
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, output_str, (text_margin,canvas.shape[1] - text_margin), font, 1,(255,255,255),1,cv2.LINE_AA)

        cv2.imshow("Visual Compass", img)
        cv2.waitKey(1)

if __name__ == '__main__':
    VisualCompassVisualization()
