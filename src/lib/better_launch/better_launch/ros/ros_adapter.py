import os
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor

"""
Provides a shared node for interacting with ROS2. Taken from ROS2.

.. seealso::
    
    `launch_ros/ros_adapters.py <https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/ros_adapters.py>`_
"""

class ROSAdapter:
    """Wraps rclpy API to ease ROS node usage in `launch_ros` actions."""

    def __init__(self, *, argv: list[str] = None, autostart: bool = True):
        """
        Construct adapter.

        :param: argv List of global arguments for rclpy context initialization.
        :param: autostart Whether to start adapter on construction or not.
        """
        # Do not use `None` here, as `rclpy.init` will use `sys.argv` in that case.
        self.argv = [] if argv is None else argv
        self.ros_context = None
        self.ros_node = None
        self.ros_executor = None
        self._thread = None

        if autostart:
            self.start()

    def start(self):
        """Start ROS adapter."""
        if self._thread and self._thread.is_alive():
            raise RuntimeError("Cannot start a ROS adapter that is already running")

        self.ros_context = rclpy.Context()
        rclpy.init(args=self.argv, context=self.ros_context)
        self.ros_node = rclpy.create_node(
            "better_launch_{}".format(os.getpid()), context=self.ros_context
        )
        self.ros_executor = SingleThreadedExecutor(context=self.ros_context)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        try:
            self.ros_executor.add_node(self.ros_node)
            self.ros_executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.ros_executor.remove_node(self.ros_node)

    def shutdown(self):
        """Shutdown ROS adapter."""
        if not self._thread or not self._thread.is_alive():
            return
        
        self.ros_executor.shutdown()
        self._thread.join()
        self.ros_node.destroy_node()
        rclpy.shutdown(context=self.ros_context)
