#!/usr/bin/env python3

from wolfgang_pybullet_sim.ros_interface import ROSInterface
from wolfgang_pybullet_sim.simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation(True)
    ros_interface = ROSInterface(simulation)
    ros_interface.run_simulation()
