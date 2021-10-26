#!/usr/bin/env python3

import argparse
import cv2
import os
import sys
import rclpy
from bitbots_ros_patches.rate import Rate
# Check if progressbar is installed
try:
    import progressbar
    progressbar_installed = True
except ImportError:
    progressbar_installed = False
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class LoadImages:
    def __init__(self, path, frame="/camera_optical_frame", topic="camera/image_proc", fps=10, loop=False, pgbar=False):
        rospy.init_node("bitbots_imageloader")
        rospy.loginfo("Started imageloader", logger_name="imageloader")

        # Register publisher
        self.pub_im = rospy.Publisher(topic, Image, queue_size=1)

        # Create CV bridge
        self.bridge = CvBridge()

        # List images in dir
        listdir = list(os.listdir(path))

        # Set fps
        rate = Rate(fps)

        # Make generator to determine if the loop runs once or infinite times
        if loop:
            # Infinite generator
            loop_generator = iter(int, 1)
        else:
            # One step generator
            loop_generator = range(1)

        # Set progressbar if we want so
        if pgbar:
            # Make progresbar iterator function
            add_progress_bar = progressbar.progressbar
        else:
            # Make normal iterator function
            add_progress_bar = lambda lst: lst

        # Iterate one or infinite times
        for i in loop_generator:
            # Iterate over all images
            for im in add_progress_bar(sorted(listdir)):
                rospy.logdebug(im, logger_name="imageloader")

                # Build path of the current image
                image_path = os.path.join(path, im)
                # Load image
                image = cv2.imread(image_path)

                # Check if this is a valid image
                if image is not None:
                    # Build image message
                    msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                    msg.header.frame_id = frame
                    msg.header.stamp = rospy.get_rostime()

                    # Publish it
                    self.pub_im.publish(msg)
                    # Wait
                    rate.sleep()
                if rospy.is_shutdown():
                    break
            if rospy.is_shutdown():
                break


if __name__ == "__main__":
    # Parse cli
    parser = argparse.ArgumentParser("Publish an image set as ROS messages.")

    parser.add_argument("-p", "--path", help="Input relative or absolute directory for the images. If none specified, current directory will be assumed", dest="path", type=str)
    parser.add_argument("-fps", "--frames-per-second", help="Playback speed.", default=10, dest="fps", type=int)
    parser.add_argument("-t", "--topic", help="ROS topic where the images are published to.", default="camera/image_proc",
        dest="topic", type=str)
    parser.add_argument("-f", "--frame", help="The tf frame where the image should be published.", default="/camera_optical_frame",
        dest="frame", type=str)
    parser.add_argument("-nl", "--no-loop", help="Set this flag, if the image sequence should not be looped.", dest="loop", action='store_false')
    parser.add_argument("-np", "--no-progressbar", help="Hides the progressbar.", dest="progressbar", action='store_false')

    args = parser.parse_args()

    # Check if a path has been set or if we should use the current working dir as path
    if args.path:
        path = args.path
    else:
        path = os.getcwd()
        print(f"Using the launch dir as path: '{path}'")

    # Check if the progressbar is installed
    if progressbar_installed:
        display_pgb = args.progressbar
    else:
        display_pgb = False
        if args.progressbar:
            print("Progressbar not installed!")

    LoadImages(path, args.frame, args.topic, args.fps, args.loop, display_pgb)
