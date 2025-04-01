import numpy as np
from soccer_vision_3d_msgs.msg import Robot


# Wrapper for the Robot message with covariance
class RobotWithCovariance(Robot):
    def __init__(self, robot: Robot, covariance: list[float] | np.ndarray = [0] * 36):
        super().__init__(attributes=robot.attributes, bb=robot.bb, confidence=robot.confidence)
        self.covariance: np.ndarray = np.array(covariance, dtype=np.float64)
        assert self.covariance.shape == (36,)
