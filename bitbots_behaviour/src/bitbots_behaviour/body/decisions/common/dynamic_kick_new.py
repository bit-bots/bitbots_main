#-*- coding:utf-8 -*-
"""
DynamicKickModule
^^^^^^^^^^^^^^^^^
Entwickelt im Robo-Cup Praktikum SoSe 2014
Weiterentwickelt im Robo-Cup Projekt WiSe 2015/16

Does the kick with inverse kinematic, based on where the ball lies.

History:
''''''''

* 22.8.14: Created (Anne-Victoria Meyer, Benjamin Scholz, Tim Dobert)
* 09.12.15: Fixed major bugs
* 06.01.16: Adaptation: improvement of kick motion (Wiese, Heinze, Stock)


"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.animation_play_action import AnimationPlayAction
from bitbots.modules.behaviour.body.actions.kinematic_task import KinematicTask

from bitbots.modules.abstract.abstract_module import AbstractModule
from bitbots.modules.keys import DATA_KEY_BALL_FOUND, DATA_KEY_BALL_INFO, \
                                DATA_KEY_IPC
from bitbots.util.speaker import say, get_config

from bitbots.util.animation import play_animation
from bitbots.robot.pypose import *
from bitbots.ipc.ipc import *
import numpy as np
from bitbots.util import get_config
from bitbots.modules.abstract.abstract_module import debug_m


class DynamicKickNew(AbstractDecisionModule):
    def __init__(self, _):
        super(DynamicKickNew, self).__init__()
        config = get_config()
        self.walkready_anim = config["animations"]["motion"]["walkready"]
        self.run = -1
        self.rebalancingrun = -1
        self.y1 = 0
        self.y2 = 0
        self.root = 0
        self.end_joint = None
        self.angle_task_joints = None
        self.ignore_joints = None
        self.angle_task_joints_extremekick = None
        self.ignore_joints_extremekick = None
        self.start = None
        self.ende = None
        self.lr = None
        self.pose = None

    def perform(self, connector, reevaluate=False):
        debug_m(4, "RUN: %f" % self.run)
        self.do_not_reevaluate()
        self.run += 1
        #connector.set_kick(True)


        if self.run == 0:
            self.first_run(connector)
            return self.push(AnimationPlayAction, "walkready")
        elif self.run == 1:
            self.run -= 1
            self.rebalancingrun += 1
            if self.rebalancingrun == 0:
                return self.push(AnimationPlayAction, "r_step")
            elif self.rebalancingrun == 1:
                return self.push(AnimationPlayAction, "fast_wr")
            elif self.rebalancingrun == 2:
                return self.push(AnimationPlayAction, "l_step")
            elif self.rebalancingrun == 3:
                self.run += 1
                self.rebalancingrun = -1
                return self.push(AnimationPlayAction, "fast_wr")
        elif self.run == 2:
            return self.push(AnimationPlayAction, self.start)
        elif self.run == 3:
            # 1. kinematische Position
            args = ({"root": self.root, "end": self.end_joint,
                     "targets": [[0, 1e-2, (1, 0, 0)], [1, 1e-2, (0, 1, 0)], [3, 1e-3, np.array((-20, self.y1, -250))]],
                     "iterations": 100},
                     0)
            return self.push(KinematicTask, args)
        elif self.run == 4:
            # 2. kinematische Position
            args = [{"root": self.root, "end": self.end_joint,
                     "targets": [[0, 1e-2, (1, 0, 0)], [1, 1e-2, (0, 1, 0)], [3, 1e-3, np.array((0, (self.y1 + self.y2)/2, -250))]],
                     "iterations": 100},
                     0]
            return self.push(KinematicTask, args)
        elif self.run == 5:
            # 3. kinematische Position
            args = [{"root": self.root, "end": self.end_joint,
                      "targets": [[0, 1e-2, (1, 0, 0)], [1, 1e-2, (0, 1, 0)], [3, 1e-3, np.array((110, self.y2, -220))]],
                      "iterations": 100},
                     0]
            return self.push(KinematicTask, args)
        elif self.run == 6:
            # 4. kinematische Position
            args = [{"root": self.root, "end": self.end_joint,
                     "targets": [[0, 1e-2, (1, 0, 0)], [1, 1e-2, (0, 1, 0)], [3, 1e-3, np.array((200, self.y2, -190))]],
                     "iterations": 100},
                     0]
            additional_angles = self.set_other_motors()
            args.append(additional_angles)
            return self.push(KinematicTask, args)
        elif self.run == 7:
            self.run -= 1
            self.rebalancingrun += 1
            if self.rebalancingrun == 0:
                return self.push(AnimationPlayAction, "r_step")
            elif self.rebalancingrun == 1:
                return self.push(AnimationPlayAction, "fast_wr")
            elif self.rebalancingrun == 2:
                return self.push(AnimationPlayAction, "l_step")
            elif self.rebalancingrun == 3:
                self.run += 1
                return self.push(AnimationPlayAction, "fast_wr")
        # elif self.run == 8:
        #     return self.push(AnimationPlayAction, self.ende)
        # elif self.run == 9:
        #     return self.push(AnimationPlayAction, self.walkready_anim)
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
            self.angle_task_joints_extremekick = [14, 16]
            self.ignore_joints_extremekick = [8, 18, 10, 12]
            self.start = "lk_start"
            self.ende = "lk_end"
        else:  # rechts
            self.lr = -1
            self.end_joint = 34
            self.angle_task_joints = [15]
            self.ignore_joints = [7, 17]
            self.angle_task_joints_extremekick = [13, 15]
            self.ignore_joints_extremekick = [7, 17, 9, 11]
            self.start = "rk_start"
            self.ende = "rk_end"

        # Berechnung der Kinematischen Positionen
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
