# -*- coding:utf-8 -*-
"""
BehaviourModule
^^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Startet das Verhalten
"""
from abstract.stack_machine_module import StackMachineModule
from body.decisions.common.duty_decider import DutyDecider
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class BehaviourModule(StackMachineModule):
    def __init__(self):
        self.set_start_module(DutyDecider)
        super(BehaviourModule, self).__init__()

        rospy.Subscriber("/odometry", Odometry, self.connector.walking.walking_callback) # todo vermutlich unnötig



        self.connector.walking.publisher = rospy.Publisher("/cmd/vel", Twist) # todo vermutlich unnörtig

        rospy.init_node("Behaviour")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    bm = BehaviourModule()
    bm.run()
