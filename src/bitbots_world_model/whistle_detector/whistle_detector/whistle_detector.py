#! /usr/bin/env python3
from typing import Optional

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Header



class WhistleDetector(Node):


    def __init__(self) -> None:
        """
        creates filter and subscribes to messages which are needed
        """
        super().__init__("ball_filter")
        self.logger = self.get_logger()
        
        self.whistle_publisher = self.create_publisher(Bool, "whistle_detected", 1)



def main(args=None) -> None:
    rclpy.init(args=args)

    node = WhistleDetector()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
