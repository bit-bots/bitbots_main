import math
from copy import deepcopy

from bitbots_splines.smooth_spline import SmoothSpline
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from sensor_msgs.msg import JointState

from bitbots_animation_server.animation import Animation


class SplineAnimator:
    def __init__(self, animation: Animation, current_joint_states: JointState, logger: Logger, clock: Clock):
        self.anim = deepcopy(animation)
        self.start_time = clock.now().nanoseconds / 1e9

        self.animation_duration: float = 0.0
        self.spline_dict: dict[str, SmoothSpline] = {}
        self.torques = {}
        self.stabilizations = {}
        self.keyframe_times: list[float] = []

        # Load keyframe positions into the splines
        current_point_time = 0.0
        for keyframe in self.anim.keyframes:
            # Calculate important times
            self.animation_duration += keyframe.duration + keyframe.pause
            current_point_time += keyframe.duration
            self.keyframe_times.append(current_point_time)
            # Add spline point for each joint in this keyframe
            for joint in keyframe.goals:
                # Check if this joint already has a spline from previous keyframes
                if joint not in self.spline_dict:
                    # If not, create a new spline
                    self.spline_dict[joint] = SmoothSpline()
                    # Add the current joint position as the first point
                    # Get the number of the joint in the joint_states message
                    joint_index = current_joint_states.name.index(joint)
                    # Add the current joint position as the point before the first keyframe
                    self.spline_dict[joint].add_point(
                        current_point_time - keyframe.duration,
                        math.degrees(current_joint_states.position[joint_index]),
                    )
                # Add the keyframe position as the next point
                self.spline_dict[joint].add_point(current_point_time, keyframe.goals[joint])
                # Add a second point if we want to hold the position for a while
                self.spline_dict[joint].add_point(current_point_time + keyframe.pause, keyframe.goals[joint])
            current_point_time += keyframe.pause
            self.torques[current_point_time] = keyframe.torque
            self.stabilizations[current_point_time] = keyframe.stabelization_functions

        # Compute the splines
        for joint in self.spline_dict:
            self.spline_dict[joint].compute_spline()

    def get_keyframe_times(self) -> list[float]:
        assert len(self.keyframe_times) == len(self.anim.keyframes)
        return self.keyframe_times

    def get_positions_deg(self, time):
        if time < 0 or time > self.animation_duration:
            return None
        ret_dict = {}
        for joint in self.spline_dict:
            ret_dict[joint] = self.spline_dict[joint].pos(time)
        return ret_dict

    def get_positions_rad(self, time):
        if time < 0 or time > self.animation_duration:
            return None
        ret_dict = {}
        for joint in self.spline_dict:
            ret_dict[joint] = math.radians(self.spline_dict[joint].pos(time))

        return ret_dict

    def get_torque(self, current_time):
        torque = {}
        if current_time < 0 or current_time > self.animation_duration:
            return torque

        # find previous time
        sorted_keys = sorted(self.torques.keys())
        keyframe_time = sorted_keys[0]
        for keyframe_time in reversed(sorted_keys):
            if keyframe_time <= current_time:
                break

        return self.torques[keyframe_time]

    def get_start_time(self):
        return self.start_time

    def get_duration(self):
        return self.animation_duration

    def get_stabalization_functions(self, current_time) -> dict[str, str]:
        if current_time < 0 or current_time > self.animation_duration:
            return {}
        # find previous time
        sorted_keys = sorted(self.stabilizations.keys())
        keyframe_time = sorted_keys[0]
        for keyframe_time in reversed(sorted_keys):
            if keyframe_time <= current_time:
                break

        return self.stabilizations[keyframe_time]
