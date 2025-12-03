import threading

import rclpy
from rclpy.experimental.events_executor import EventsExecutor

from bitbots_mujoco_sim.simulation import Simulation


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
