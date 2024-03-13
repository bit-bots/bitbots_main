import math

from bitbots_splines.smooth_spline import SmoothSpline
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from sensor_msgs.msg import JointState

from bitbots_animation_server.animation import Animation


class SplineAnimator:
    def __init__(self, animation: Animation, current_joint_states: JointState, logger: Logger, clock: Clock):
        self.anim = animation
        self.start_time = clock.now().nanoseconds / 1e9

        self.animation_duration = 0.0
        self.current_point_time = 0.0
        self.spline_dict: dict[str, SmoothSpline] = {}
        self.torques = {}
        self.keyframe_times: list[float] = []

        # add current joint positions as start
        if current_joint_states is not None:
            i = 0
            for joint in current_joint_states.name:
                if joint not in self.spline_dict:
                    self.spline_dict[joint] = SmoothSpline()
                self.spline_dict[joint].add_point(0, math.degrees(current_joint_states.position[i]))
                i += 1
        else:
            logger.warn("No current joint positions. Will play animation starting from first keyframe.")
            for joint in self.anim.keyframes[0].goals:
                if joint not in self.spline_dict:
                    self.spline_dict[joint] = SmoothSpline()
                self.spline_dict[joint].add_point(0, self.anim.keyframes[0].goals[joint])

        # load keyframe positions into the splines
        for keyframe in self.anim.keyframes:
            self.animation_duration += keyframe.duration + keyframe.pause
            self.current_point_time += keyframe.duration
            self.keyframe_times.append(self.current_point_time)
            for joint in keyframe.goals:
                if joint not in self.spline_dict:
                    self.spline_dict[joint] = SmoothSpline()
                self.spline_dict[joint].add_point(self.current_point_time, keyframe.goals[joint])
                self.spline_dict[joint].add_point(self.current_point_time + keyframe.pause, keyframe.goals[joint])
            self.current_point_time += keyframe.pause
            self.torques[self.current_point_time] = keyframe.torque

        # compute the splines
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
