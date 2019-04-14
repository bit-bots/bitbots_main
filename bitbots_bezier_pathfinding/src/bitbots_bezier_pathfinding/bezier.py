import math
import numpy

from geometry_msgs.msg import Pose2D
# The bezier class models a bezier curve
# and provides class methods to generate them.

class Bezier:
    LEFT = 1
    RIGHT = -1
    FORWARD = 0
    BACKWARDS = 1
    SIDEWAYS = 2

    def __init__(self, start_position, control_1, control_2, target_position, direction):
        self.start_position = start_position
        self.control_1 = control_1
        self.control_2 = control_2
        self.target_position = target_position
        self.direction = direction

    def get_xy(self, t):
        # t is between 0 and 1
        # This is just the default cubic bezier formula
        x = ((1 - t) * (1 - t) * (1 - t) * self.start_position.x
             + 3 * (1 - t) * (1 - t) * t * self.control_1.x
             + 3 * (1 - t) * t * t * self.control_2.x
             + t * t * t * self.target_position.x)
        y = ((1 - t) * (1 - t) * (1 - t) * self.start_position.y
             + 3 * (1 - t) * (1 - t) * t * self.control_1.y
             + 3 * (1 - t) * t * t * self.control_2.y
             + t * t * t * self.target_position.y)
        return x, y

    def get_direction(self, t, stepsize):
        # t is between 0 and 1
        # The direction is calculated from the position of the point
        # at time t to the position of the point at time t+stepsize
        first_point = self.get_xy(t)
        second_point = self.get_xy(t + stepsize)
        theta = math.atan2(second_point[1] - first_point[1], second_point[0] - first_point[0])
        return theta

    def get_time_at_distance(self, distance, stepsize):
        current_distance = 0
        current_time = 0
        old_x, old_y = self.get_xy(current_time)
        while current_distance < distance:
            current_time += stepsize
            if current_time > 1:
                return 1
            new_x, new_y = self.get_xy(current_time)
            current_distance += math.sqrt((old_x - new_x)**2 + (old_y - new_y)**2)
            old_x, old_y = new_x, new_y
        return current_time
    
    def get_distance_at_time(self, time, stepsize):
        current_distance = 0
        current_time = 0
        old_x, old_y = self.get_xy(current_time)
        while current_time < time:
            current_time += stepsize
            new_x, new_y = self.get_xy(current_time)
            current_distance += math.sqrt((old_x - new_x)**2 + (old_y - new_y)**2)
            old_x, old_y = new_x, new_y
        return current_distance

    def draw(self):
        import pylab
        ts = numpy.linspace(0, 1, 100)
        x, y = self.get_xy(ts)
        pylab.scatter(x, y)
        pylab.show()

    @classmethod
    def from_pose(cls, target_pose, straightness):
        my_position = Pose2D(0, 0, 0)
        target_position = target_pose
        control_distance = math.sqrt(target_pose.x ** 2 + target_pose.y ** 2) * straightness
        control_1 = Pose2D()
        control_1.x = control_distance
        control_1.y = 0
        control_2 = Pose2D()
        control_2.x = target_pose.x - control_distance * math.cos(target_pose.theta)
        control_2.y = target_pose.y - control_distance * math.sin(target_pose.theta)
        return Bezier(my_position, control_1, control_2, target_position, Bezier.FORWARD)

    @classmethod
    def backwards(cls, direction):
        control = Pose2D()
        control.x = -0.1
        control.y = 0
        target_pose = Pose2D()
        target_pose.x = -1
        target_pose.y = math.sin(direction * math.pi / 2)
        return Bezier(Pose2D(0, 0, 0), control, control, target_pose, Bezier.BACKWARDS)

    @classmethod
    def sideways(cls, target_pose):
        control_1 = Pose2D()
        control_1.x = 0
        control_1.y = target_pose.y / 2
        control_2 = Pose2D()
        control_2.x = target_pose.x
        control_2.y = target_pose.y / 2
        return Bezier(Pose2D(0, 0, 0), control_1, control_2, target_pose, Bezier.SIDEWAYS)
