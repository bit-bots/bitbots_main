import threading

import rclpy
from bitbots_utils.perf_timer import configure_output
from rclpy.experimental.events_executor import EventsExecutor

from bitbots_mujoco_sim.simulation import Simulation


def main(args=None):
    rclpy.init(args=args)
    configure_output("/tmp/mujoco_perf", "MuJoCo")
    simulation = Simulation()
    executor = EventsExecutor()
    executor.add_node(simulation)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    simulation.run()
    simulation.destroy_node()
    rclpy.shutdown()
