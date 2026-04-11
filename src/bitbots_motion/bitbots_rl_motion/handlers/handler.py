from abc import ABC, abstractmethod

from rclpy.node import Node


class Handler(ABC, Node):
    def __init__(self, config):
        self._config = config

    @abstractmethod
    def has_data(self):
        pass
