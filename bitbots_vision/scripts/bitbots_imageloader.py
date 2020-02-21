#!/usr/bin/env python3

import argparse
import cv2
import os
import sys
import rospy
try:
    import progressbar
    progressbar_installed = True
except ImportError:
    progressbar_installed = False
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class LoadImages:
    def __init__(self, path, frame="/camera_link", topic="image_raw", fps=10, loop=False, pgbar=False):
        rospy.init_node("bitbots_imageloader")
        rospy.loginfo("Started imageloader", logger_name="imageloader")
        self.pub_im = rospy.Publisher(topic, Image, queue_size=1)
        self.bridge = CvBridge()

        listdir = list(os.listdir(path))

        rate = rospy.Rate(fps)

        if loop:
            loop_generator = iter(int, 1)
        else:
            loop_generator = range(1)

        if pgbar:
            add_progress_bar = lambda lst: progressbar.progressbar(lst)
        else:
            add_progress_bar = lambda lst: lst

        for i in loop_generator:
            for im in add_progress_bar(sorted(listdir)):
                rospy.logdebug(im, logger_name="imageloader")

                image_path = os.path.join(path, im)

                image = cv2.imread(image_path)

                msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                msg.header.frame_id = "/camera_link"
                msg.header.stamp = rospy.get_rostime()

                self.pub_im.publish(msg)

                if rospy.is_shutdown():
                    break
                rate.sleep()
            if rospy.is_shutdown():
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Publish a image set as ros messages.")

    parser.add_argument("-p", "--path", help="Input directory for the images", dest="path", type=str)
    parser.add_argument("-fps", "--frames-per-second", help="Playback speed.", default=10, dest="fps", type=int)
    parser.add_argument("-t", "--topic", help="ROS topic where the images are published to.", default="image_raw",
        dest="topic", type=str)
    parser.add_argument("-f", "--frame", help="The tf frame where the image should be published.", default="/camera_link",
        dest="frame", type=str)
    parser.add_argument("-nl", "--no-loop", help="Should the image sequence not be looped.", dest="loop", action='store_false')
    parser.add_argument("-np", "--no-progressbar", help="Hides the progressbar.", dest="progressbar", action='store_false')

    args = parser.parse_args()

    if args.path:
        path = args.path
    else:
        path = os.getcwd()
        print("Using the launch dir as path: {}".format(path))

    if progressbar_installed:
        display_pgb = args.progressbar
    else:
        display_pgb = False
        if args.progressbar:
            print("Progressbar not installed!")

    LoadImages(path, args.frame, args.topic, args.fps, args.loop, display_pgb)
