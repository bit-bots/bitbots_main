# -*- coding:utf-8 -*-
"""
DynamicKick
^^^^^^^^^^^

Does the kick with inverse kinematic, based on where the ball lies.

History:
* 08.12.14: Created (Marc Bestmann)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.animation_play_action import AnimationPlayAction
from bitbots.modules.behaviour.body.actions.kinematic_task import KinematicTask
import numpy as np
from bitbots.util import get_config


class DynamicKick(AbstractDecisionModule):
    def __init__(self, _):
        super(DynamicKick, self).__init__()
        config = get_config()
        self.walkready_anim = config["animations"]["motion"]["walkready"]
        self.run = -1
        self.x = 0
        self.y1 = 0
        self.y2 = 0
        self.root = 0
        self.end_joint = None
        self.angle_task_joints = None
        self.ignore_joints = None
        self.start = None
        self.ende = None
        self.lr = None
        self.pose = None

    def perform(self, connector, reevaluate=False):
        self.do_not_reevaluate()
        self.run += 1

        if self.run == 0:
            self.first_run(connector)
            return self.push(AnimationPlayAction, self.start)
        elif self.run == 1:
            # 1. kinematische Position
            args = ({"root": self.root, "end": self.end_joint,
                     "targets": [[0, 1e-2, (1, 0, 0)], [3, 1e-3, np.array((0, self.y1, -280))]],
                     "iterations": 100},
                     0)
            return self.push(KinematicTask, args)
        elif self.run == 2:
            # 2. kinematische Position
            args = [{"root": self.root, "end": self.end_joint,
                     "targets": [[0, 1e-2, (1, 0, 0)], [3, 1e-3, np.array((self.x, self.y2, -240))]],
                     "iterations": 100},
                     0]
            additional_angles = self.set_other_motors()
            args.append(additional_angles)
            return self.push(KinematicTask, args)
        elif self.run == 3:
            return self.push(AnimationPlayAction, self.ende)
        elif self.run == 4:
            return self.push(AnimationPlayAction, self.walkready_anim)
        else:
            return self.interrupt()

    def first_run(self, connector):
        ball_v = connector.raw_vision_capsule().get_ball_info("v")

        param = abs(ball_v) - 50
        if param > 100:
            param = 100
        if param < 0:
            param = 0

        if ball_v > 0:  # links
            self.lr = 1
            self.end_joint = 35
            self.angle_task_joints = [16]
            self.ignore_joints = [8, 18]
            self.start = "lk_start"
            self.ende = "lk_end"
        else:  # rechts
            self.lr = -1
            self.end_joint = 34
            self.angle_task_joints = [15]
            self.ignore_joints = [7, 17]
            self.start = "rk_start"
            self.ende = "rk_end"

        # Berechnung der Kinematischen Positionen
        self.x = 140 - 0.4 * param
        self.y1 = self.lr * (40 + 0.5 * param)
        self.y2 = self.y1 + self.lr * (0.05 * param)  # Für Stabilität vielleicht y2 = y1

    def set_other_motors(self):
        if self.lr == 1:
            additional_angles = [
                ("LShoulderPitch", 61.69921875),
                ("LElbow", -0.703125),
                ("RKnee", 55.283203125),
                ("RAnkleRoll", -9.4921875),
                ("RAnklePitch", 27.421875),
                ("RHipYaw", -0.703125),
                ("RHipRoll", 2.8125),
                ("RHipPitch", -40.517578125),
                ("RShoulderPitch", -15.64453125),
                ("RElbow", 49.39453125)]
        else:
            additional_angles = [
                ("RShoulderPitch", -61.69921875),
                ("RElbow", 0.703125),
                ("LKnee", -55.283203125),
                ("LAnkleRoll", 9.4921875),
                ("LAnklePitch", -27.421875),
                ("LHipYaw", 0.703125),
                ("LHipRoll", -2.8125),
                ("LHipPitch", 40.517578125),
                ("LShoulderPitch", 15.64453125),
                ("LElbow", -49.39453125)]

        return additional_angles
