#! /usr/bin/env python2
import os
import rospy
import rospkg
import argparse
# Use ruamel.yaml to be comment/order persistent
from ruamel.yaml import YAML

"""
A script to save dynamic reconfigured params into the visionparams.yaml
"""

# Parameters for debug purposes that should be ignored if the -id or --ignore-debug flag is set
debug_params = [
    'ball_fcnn_publish_debug_img',
    'vision_publish_debug_image',
    'vision_publish_field_mask_image',
    'dynamic_color_space_publish_field_mask_image',
]

# Compatibility
try:
    input = raw_input
except:
    pass

# Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("-i", "--ignore-debug", help="Ignore debug params", action='store_true')
args = parser.parse_args()

# Set yaml stuff
yaml = YAML()
yaml.indent(mapping=3)
yaml.preserve_quotes = True

# Get path information
rospack = rospkg.RosPack()
package_path = rospack.get_path('bitbots_vision')
config_path = os.path.join(package_path, "config", "visionparams.yaml")

# Open old config file
with open(config_path) as fp:
    data = yaml.load(fp)

changed_params = 0
# Iterate over old keys
for key in data.keys():
    # Get the current key value from the parameter server
    new_value = rospy.get_param("/bitbots_vision/{}".format(str(key)))
    # Check if param changed or if it's a debug parameter that should be ignored
    if new_value != data[key] and (not args.ignore_debug or not key in debug_params):
        data[key] = new_value
        changed_params += 1
        print("{}:{}".format(key, new_value))

# Ask user if he wants to save it
if input("\n {} parameters changed. Do you want to save? (y/n)".format(changed_params)) == "y":
    # Save new file
    with open(config_path,"w") as fp:
        yaml.dump(data, fp)
    print("Saved file")
else:
    print("Nothing has been saved!")
