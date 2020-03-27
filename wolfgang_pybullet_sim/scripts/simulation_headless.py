#!/usr/bin/env python3

from wolfgang_pybullet_sim.ros_interface import ROSInterface
from wolfgang_pybullet_sim.simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation(False)
    ros_interace = ROSInterface(simulation)
    ros_interace.run_simulation()
