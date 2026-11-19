import threading

import rclpy
from bitbots_utils.perf_timer import configure_output
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from bitbots_mujoco_sim.simulation import Simulation


def main(args=None):
    rclpy.init(args=args)
    configure_output("/tmp/mujoco_perf", "MuJoCo")
    simulation = Simulation()
    # The teleport service awaits a physics-thread acknowledgement.
    # Use an executor that supports coroutine service callbacks on Jazzy.
    executor = SingleThreadedExecutor()
    executor.add_node(simulation)

    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        simulation.run()
    finally:
        # A paused viewer may still have an outstanding teleport coroutine.
        executor.shutdown(timeout_sec=0.0)
        rclpy.try_shutdown()
        thread.join()
        simulation.destroy_node()
