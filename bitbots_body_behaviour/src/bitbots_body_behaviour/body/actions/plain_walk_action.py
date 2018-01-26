"""
PlainWalkAction
^^^^^^^^^^^^^^^

.. moduleauthor:: Sheepy


The robot performs a cretain set of walking parameters. The action expects a list of 4-tuples as first argument.

"""
import rospy

from bitbots_stackmachine.abstract_init_action_module import AbstractInitActionModule

from bitbots_common.connector.connector import BodyConnector


class PlainWalkAction(AbstractInitActionModule):
    def __init__(self, connector: BodyConnector, args):
        AbstractInitActionModule.__init__(self, connector, args)
        self.step_list = args
        self.walking_steps = self.walking_step_generator_function()
        self.active = False

    def walking_step_generator_function(self):
        """ This is a coroutine for head movement """
        if self.step_list is None:
            raise Exception("Step list may not be None.")
        for element in self.step_list:
            yield element
        yield None

    def perform(self, connector: BodyConnector, reevaluate=False):
        if connector.blackboard.get_duty() in ["ThrowIn", "PenaltyKickFieldie"]:
            self.do_not_reevaluate()
        if not self.active:
            self.current_step = next(self.walking_steps)
            if self.current_step is None:
                connector.walking.stop_walking()
                return self.pop()
            f, a, s, t = self.current_step
            self.active = True
            self.started_time = rospy.get_time()

            connector.walking.start_walking_plain(f, a, s)
        else:
            if rospy.get_time() > self.current_step[3] + self.started_time:
                self.active = False
