#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from bitbots_msgs.srv import ManualPenalize
from bitbots_msgs.msg import Audio
from humanoid_league_speaker.speaker import speak


class Pause(object):
    """
    Always go in and out of pause by manual penalty.
    Go in and out of pause by game controller, if manual penalty is not active.
    """

    def __init__(self):
        rclpy.init(args=None)
        self.node = Node("Pause")

        self.penalty_manual = False
        self.game_controller_penalty = False
        self.pause = False

        self.manual_penalize_service = self.node.create_service(ManualPenalize, "manual_penalize", self.manual_update)
        self.pause_publisher = self.node.create_publisher(Bool, "pause", 10)  # todo latch
        self.speak_publisher = self.node.create_publisher(Audio, "speak", 10)

        rclpy.spin(self.node)

    def manual_update(self, req: ManualPenalize.Request, resp: ManualPenalize.Response):
        if req.penalize == ManualPenalize.Request.OFF:
            self.penalty_manual = False
        elif req.penalize == ManualPenalize.Request.ON:
            self.penalty_manual = True
        elif req.penalize == ManualPenalize.Request.SWITCH:
            self.penalty_manual = not self.penalty_manual
        else:
            self.node.get_logger().error("Manual penalize call with unspecified request")
        self.set_pause(self.penalty_manual)
        return resp

    def set_pause(self, state):
        self.pause = state
        if state:
            text = "Pause!"
        else:
            text = "Unpause!"
        self.node.get_logger().warn(text)
        speak(text, self.speak_publisher, speaking_active=True, priority=90)
        self.pause_publisher.publish(Bool(data=state))


if __name__ == '__main__':
    pause = Pause()
