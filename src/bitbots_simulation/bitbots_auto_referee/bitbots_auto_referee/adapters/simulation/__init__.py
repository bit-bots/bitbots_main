"""Future ROS transport to the separately running simulator.

The simulator must export consistent observations and acknowledge placement
commands. MuJoCo data cannot be accessed directly across this process boundary.
"""
