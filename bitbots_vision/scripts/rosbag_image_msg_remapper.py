#!/usr/bin/env python3

from rosbag import Bag
import sys
import os
from os import listdir

"""
A script that remaps the /image_raw topic in rosbags to /camera/image_proc.

Usage:
python rosbag_image_msg_remapper.py [input file/directory]
"""


def remap(input_bag_path, output_bag_path):
    """
    remaps the topics of a single bag
    :param input_bag_path: the bag with the topic that should be renamed
    :param output_bag_path: the path where the output bag should be created
    """
    with Bag(output_bag_path, 'w') as output_bag:  # create output bag
        # write all topics into the output bag and rename the specified one:
        for topic, msg, t in Bag(input_bag_path):
            if topic == '/image_raw':
                output_bag.write('/camera/image_proc', msg, t)
            else:
                output_bag.write(topic, msg, t)


# Remap topic:
print('This may take a while...')
input_path = sys.argv[1]  # path of the input directory or rosbag
if os.path.isdir(input_path):  # input is a directory
    # create output directory:
    output_path = input_path + '_updated'
    os.mkdir(output_path)
    # remap topic in every rosbag in the input directory:
    for input_bag in listdir(sys.argv[1]):
        print("remapping " + input_bag)
        remap(
            input_path + '/' + input_bag,
            output_path + '/' + input_bag[:-4] + '_updated.bag'
        )

else:  # input is a file
    output_path = input_path[:-4] + '_updated.bag'  # path for the output rosbag
    remap(
        input_path,
        output_path
    )


