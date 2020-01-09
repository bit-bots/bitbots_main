#!/usr/bin/env python3

import os
import cv2
import rosbag
import argparse
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def yes_or_no_input(question, default=None):
    # type: (str) -> bool
    """
    Prints a yes or no question and returns the answer.
    
    :param str question: Yes or no question
    :param bool default: Default answer, if empty answer
    :return bool: Input answer
    """
    answer = None
    reply = None
    extension = None

    if default is None:
        extension = " [y|n]"
    elif default == True:
        extension = " [Y|n]"
    elif default == False:
        extension = " [y|N]"

    while answer is None:
        reply = str(input(question + extension + ": ")).lower().strip()
        if default is not None and reply == "":
            answer = default
        elif reply[:1] == 'y':
            answer = True
        elif reply[:1] == 'n':
            answer =  False
    return answer

def int_input(question, min_int=None, max_int=None):
    # type: (str, int, int) -> int
    """
    Prints a question about a int value and returns the answer.
    
    :param str question: Int question
    :param int min_int: Minimum input value to be accepted
    :param int max_int: Maximum input value to be accepted
    :return int: Input answer
    """
    answer = None
    reply = None
    extension = None

    # Construct full question with min and max
    if min_int is not None and max_int is None:
        extension = " [MIN: {}]".format(min_int)
    elif min_int is None and max_int is not None:
        extension = " [MAX: {}]".format(max_int)
    elif min_int is not None and max_int is not None:
        if not min_int <= max_int:
            raise ValueError("min_int must be smaller or equal to max_int.")
        else:
            extension = " [{} - {}] ".format(min_int, max_int)

    while answer is None:
        try:
            reply = int(input(question + extension + ": ").strip())
        except ValueError:
            pass

        # Check for min and max conditions
        if reply is not None:
            if min_int is None and max_int is None or \
                    min_int is not None and max_int is None and min_int <= reply or \
                    min_int is None and max_int is not None and reply >= max_int or \
                    min_int is not None and max_int is not None and min_int <= reply <= max_int:
                answer = reply
    return answer

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
    print("Error while opening bag")
    exit(1)

topics_and_type = bag.get_type_and_topic_info()
image_topics_and_info = []

for topic, info in topics_and_type.topics.items():
    if info.msg_type == "sensor_msgs/Image":
        image_topics_and_info.append([topic, info])

if len(image_topics_and_info) == 0:  # 0 topic found
    print("No messages of type sensor_msgs/Image found in the provided rosbag")
    exit()
elif len(image_topics_and_info) == 1:  # 1 topic found
    print(
        "Found exactly one topic ({0}) of type sensor_msgs/Image with {1} messages".format(
            image_topics_and_info[0][0],
            image_topics_and_info[0][1].message_count))
    if image_topics_and_info[0][0] == args.topic:
        chosen_set_num = 0
    else:
        selection = yes_or_no_input("Do you want to extract images from this topic?", default=True)
        if not selection:
            exit()
        chosen_set_num = 0
else:  # Multiple topics found
    if args.topic in image_topics_and_info:  # Topic already specified in argument
        for i, topic_tuple in enumerate(image_topics_and_info):
            if topic_tuple[0] == args.topic:
                chosen_set = image_topics_and_info[i]
    else:  # Multiple topics, but not specified yet
        print("Multiple topics with type sensor_msgs/Image:")
        for i, topic_tuple in enumerate(image_topics_and_info):
            print("[" + str(i) + "] topic: " + str(topic_tuple[0]) + " \t message_count: " + str(
                topic_tuple[1].message_count))
        chosen_set_num = int_input("Make a selection", min_int=0, max_int=len(image_topics_and_info) - 1)

chosen_set = image_topics_and_info[chosen_set_num]

print("The dataset you have selected has a frequency of {0}".format(chosen_set[1].frequency))
if args.n is None:
    n = int_input("Every n-th image will be saved. Please specify n", min_int=1)
else:
    n = args.n

print("Extracting every {}-th image.".format(n))

try:
    os.mkdir(args.outputdir)
except OSError:
    if not yes_or_no_input("Directory already exists, continue?" ,default=True):
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
    cv2.imwrite(args.outputdir + "/img{:05d}".format(frame_number) + ".png", cv_image)
    frame_number += 1
    if frame_number % 10 == 0:
        print("\r{}/{}".format(frame_number, chosen_set[1].message_count // n), end="")

print("\r{}/{}".format(frame_number, chosen_set[1].message_count // n))

print("Image extraction complete.")
