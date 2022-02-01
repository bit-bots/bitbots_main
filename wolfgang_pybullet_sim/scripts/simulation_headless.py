#!/usr/bin/env python3
import threading

import rclpy
from wolfgang_pybullet_sim.ros_interface import ROSInterface
from wolfgang_pybullet_sim.simulation import Simulation

if __name__ == "__main__":
    rclpy.init(args=None)
    simulation = Simulation(False)
    node = ROSInterface(simulation)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run_simulation()

    node.destroy_node()
    rclpy.shutdown()
