This package provides a PyBullet simulation environment with ROS topic support for the Wolfgang Robot.

There are different options to use this:
1. Start the simulation with interface `rosrun wolfgang_pybullet_sim simulation_with_gui.py`
2. Start the simulation without interface `rosrun wolfgang_pybullet_sim simulation_headless.py`
3. Use the python class `Simulation` in `simulation.py` to directly run a simulation without using ROS

Shortcuts in gui:

`r` reset simulation

`g` debug interface

`space` pausing

`s` hold to step while pausing

`n` gravity on/off