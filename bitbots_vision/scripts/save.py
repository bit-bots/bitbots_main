#! /usr/bin/env python2
import os
import rospy
import rospkg
# Use ruamel.yaml to be comment/order persistent
from ruamel.yaml import YAML

"""
A script to save dynamic reconfigured params into the visionparams.yaml
"""

# Compatibility
try:
    input = raw_input
except:
    pass

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
    # Check if param changed
    if new_value != data[key]:
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
