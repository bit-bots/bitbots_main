from turtle import pos
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from rclpy.duration import Duration
from bitbots_msgs.msg import JointCommand
from math import radians


def parse_dict_to_msg(joint_dict: dict, time_stamp):
    names = []
    positions = []
    for name, value in joint_dict.items():
        names.append(name)
        positions.append(radians(value))
    msg = JointCommand()
    msg.joint_names = names
    msg.positions = positions
    msg.velocities = len(names) * [-1.0]
    msg.accelerations = len(names) * [-1.0]
    msg.max_currents = len(names) * [-1.0]
    msg.header.stamp = time_stamp
    return msg


class AbstractFallingPose(AbstractActionElement):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.first_perform = True
        self.start_time = blackboard.node.get_clock().now()
        self.duration = parameters.get('duration', 1.0)
        self.falling_pose = None

    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        if self.first_perform:
            self.first_perform = False
            self.blackboard.joint_pub.publish(self.falling_pose)
        # wait some time
        if self.blackboard.node.get_clock().now() - self.start_time > Duration(seconds=self.duration):
            self.pop()


class FallingPoseFront(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0.0,
            "HeadTilt": 45.0,
            "LAnklePitch": -36.0,
            "LAnkleRoll": 4.0,
            "LElbow": 45.0,
            "LHipPitch": -11.0,
            "LHipRoll": 4.0,
            "LHipYaw": 6.0,
            "LKnee": 13.0,
            "LShoulderPitch": 90.0,
            "LShoulderRoll": 0.0,
            "RAnklePitch": 36.0,
            "RAnkleRoll": -4.0,
            "RElbow": -45.0,
            "RHipPitch": 11.0,
            "RHipRoll": -4.0,
            "RHipYaw": 6.0,
            "RKnee": -13.0,
            "RShoulderPitch": -90.0,
            "RShoulderRoll": 0.0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict, self.blackboard.node.get_clock().now().to_msg())


class FallingPoseBack(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0.0,
            "HeadTilt": 0.0,
            "LAnklePitch": -29.0,
            "LAnkleRoll": 0.0,
            "LElbow": 47.0,
            "LHipPitch": 51.0,
            "LHipRoll": 0.0,
            "LHipYaw": 0.0,
            "LKnee": 64.0,
            "LShoulderPitch": 1.0,
            "LShoulderRoll": 0.0,
            "RAnklePitch": 28.0,
            "RAnkleRoll": -4.0,
            "RElbow": -45.0,
            "RHipPitch": -50.0,
            "RHipRoll": -1.0,
            "RHipYaw": 1.0,
            "RKnee": -61.0,
            "RShoulderPitch": 0.0,
            "RShoulderRoll": 0.0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict, self.blackboard.node.get_clock().now().to_msg())


class FallingPoseLeft(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0.0,
            "HeadTilt": 0.0,
            "LAnklePitch": -26.0,
            "LAnkleRoll": 4.0,
            "LElbow": 44.0,
            "LHipPitch": 27.0,
            "LHipRoll": 4.0,
            "LHipYaw": -1.0,
            "LKnee": 58.0,
            "LShoulderPitch": -2.0,
            "LShoulderRoll": 0.0,
            "RAnklePitch": 26.0,
            "RAnkleRoll": -4.0,
            "RElbow": -42.0,
            "RHipPitch": -27.0,
            "RHipRoll": -4.0,
            "RHipYaw": 1.0,
            "RKnee": -58.0,
            "RShoulderPitch": 6.0,
            "RShoulderRoll": 0.0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict, self.blackboard.node.get_clock().now().to_msg())


class FallingPoseRight(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0.0,
            "HeadTilt": 0.0,
            "LAnklePitch": -26.0,
            "LAnkleRoll": 4.0,
            "LElbow": 44.0,
            "LHipPitch": 27.0,
            "LHipRoll": 4.0,
            "LHipYaw": -1.0,
            "LKnee": 58.0,
            "LShoulderPitch": -2.0,
            "LShoulderRoll": 0.0,
            "RAnklePitch": 26.0,
            "RAnkleRoll": -4.0,
            "RElbow": -42.0,
            "RHipPitch": -27.0,
            "RHipRoll": -4.0,
            "RHipYaw": 1.0,
            "RKnee": -58.0,
            "RShoulderPitch": 6.0,
            "RShoulderRoll": 0.0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict, self.blackboard.node.get_clock().now().to_msg())
