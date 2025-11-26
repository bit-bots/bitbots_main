import math

import rclpy

from bitbots_mujoco_sim.Simulation import Simulation
import threading
from rclpy.experimental.events_executor import EventsExecutor

def main(args=None):
    rclpy.init(args=args)
    simulation = Simulation()
    executor = EventsExecutor()
    executor.add_node(simulation)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    simulation.run()
    simulation.destroy_node()
    rclpy.shutdown()
