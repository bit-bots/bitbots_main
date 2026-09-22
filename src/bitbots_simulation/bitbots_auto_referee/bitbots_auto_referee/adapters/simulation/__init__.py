"""ROS observation transport to the separately running simulator.

Positions and contacts arrive together in simulation snapshots. Future placement
commands need acknowledgements; MuJoCo memory is not shared across processes.
"""
