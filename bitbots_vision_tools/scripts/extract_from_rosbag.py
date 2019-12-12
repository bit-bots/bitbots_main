#!/usr/bin/env python

import rosbag
import argparse
import os
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from cv_bridge import CvBridge
import numpy as np

try:
    input = raw_input
except NameError:
    pass

parser = argparse.ArgumentParser("Extract images from a rosbag")

parser.add_argument("-o", "--out-dir", required=True, help="Output directory for the images", dest="outputdir")
parser.add_argument("-i", "--input-bag", required=True, help="Input rosbag", dest="inputfile")
parser.add_argument("-n", "--nth-image", help="Every n-th image will be saved, prompted if not specified", dest="n",
                    type=int)
parser.add_argument("-t", "--topic", help="Image topic, prompted if not asked", dest="topic")

args = parser.parse_args()
try:
    bag = rosbag.Bag(args.inputfile, "r")
except IOError:
    print("error opening bag")
    exit(1)

topics_and_type = bag.get_type_and_topic_info()
image_topics_and_info = []

for topic, info in topics_and_type.topics.items():
    if info.msg_type == "sensor_msgs/Image":
        image_topics_and_info.append([topic, info])

print(image_topics_and_info)
if len(image_topics_and_info) == 0:
    print("No messages of type sensor_msgs/Image found in the provided rosbag")
    exit()
elif len(image_topics_and_info) == 1:
    print(
        "Found exactly one topic ({0}) of type sensor_msgs/Image with {1} messages".format(image_topics_and_info[0][0],
                                                                                           image_topics_and_info[0][
                                                                                               1].message_count))
    if image_topics_and_info[0][0] == args.topic:
        chosen_set_num = 0
    else:
        selection = input("Do you want to extract images from this topic? y/n: ")
        if selection != "y":
            exit()
        chosen_set_num = 0
else:
    print("Multiple topics with type sensor_msgs/Image:")
    specified_topic_in_topics = False
    for i, topic_touple in enumerate(image_topics_and_info):
        print("[" + str(i) + "] topic: " + str(topic_touple[0]) + " \t message_count: " + str(
            topic_touple[1].message_count))
        if topic_touple[0]:
            chosen_set_num = i
            specified_topic_in_topics = True
    if not specified_topic_in_topics:
        try:
            chosen_set_num = int(input("Make a selection [0-{}] or q: ".format(len(image_topics_and_info) - 1)))
        except ValueError:
            exit()

chosen_set = image_topics_and_info[chosen_set_num]

print("The dataset you have selected has a frequency of {0}".format(chosen_set[1].frequency))
if args.n is None:
    try:
        n = int(input("Every n-th image will be saved. Please specify n:"))
    except ValueError:
        exit()
else:
    n = args.n

print("Extracting every {}th image".format(n))

try:
    os.mkdir(args.outputdir)
except OSError:
    answer = input("Directory already exists, continue? y/n: ")
    if answer != "y":
        exit()

i = 0
frame_number = 0
bridge = CvBridge()
for bag_msg in bag.read_messages(chosen_set[0]):
    i = (i + 1) % n
    if i != 0:
        continue
    msg_from_bag = bag_msg.message
    img = Image()
    img.header = msg_from_bag.header
    img.data = msg_from_bag.data
    img.height = msg_from_bag.height
    img.width = msg_from_bag.width
    img.encoding = msg_from_bag.encoding
    img.is_bigendian = msg_from_bag.is_bigendian
    img.step = msg_from_bag.step

    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    cv2.imwrite(args.outputdir + "/img{:05d}".format(frame_number) + ".png", cv_image, )
    frame_number += 1
    if frame_number % 10 == 0:
        print("{}/{}".format(frame_number, chosen_set[1].message_count / n))

print("{}/{}".format(frame_number, chosen_set[1].message_count / n))

print("Image extraction complete.")
