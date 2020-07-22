#!/usr/bin/env python3

from rosbag import Bag
import sys

"""
A script that remaps the image_raw topic in rosbags to image_proc
"""

# Organize arguments into array:
arguments = []
for argument in sys.argv:
    # strip the .bag from the name:
    if argument.endswith('.bag'):
        argument = argument[:-4]
    arguments.append(str(argument))

# Check if enough arguments are given:
if len(arguments) == 1:
    # helping terminal output if two few arguments are given:
    print('Usage: python '
          + __file__ +
          ' <input_bag_path> '
          '[<output_bag_path>] ')
    exit()

# Set input bag name:
input_bag_path = arguments[1]

# Set output bag name:
if len(arguments) == 3:
    # output bag name is specified
    output_bag_path = arguments[2]
else:
    # output bag name is not specified
    output_bag_path = input_bag_path + '_updated'

# Remap topic:
print('This may take a while...')
with Bag(output_bag_path + '.bag', 'w') as output:  # create output bag
    # write all topics into the output bag and rename them if necessary:
    for topic, msg, t in Bag(input_bag_path + '.bag'):
        if topic == '/image_raw':
            output.write('/image_proc', msg, t)
        else:
            output.write(topic, msg, t)
