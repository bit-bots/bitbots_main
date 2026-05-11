from rclpy import logging


def get_logger():
    return logging.get_logger("dynamic_stack_decider")
