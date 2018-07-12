# -*- coding:utf-8 -*-
import json
from bitbots_stackmachine.abstract_stack_element import AbstractStackElement


class AbstractActionElement(AbstractStackElement):
    """
        One action is similar to a state of an FSM. 
        As in this case, the system stays in this state in contrast to the decision elements which are called only for determining the active action.
        It defines the actions which the robot does, for example performing a kick.
        Another example is an action which takes care of going to the ball, the action remains on top of the stack until the ball is reached.
        The action only makes decisions which are necessary for its purpose, like some adjustments to the kicking movement.
        Actions do not push further elements on the stack but command actions on lower-level modules like new movement goals.
        If the action is complete, it can remove itself from the stack by performing a pop command.     
    """

    def __repr__(self):
        """
            Overlaod from the AbstractStackElement to have "Action" at the start.
        """
        shortname = self.__class__.__module__.split('.')[-2] \
                    + "." + self.__class__.__name__

        data = json.dumps(self.repr_data)

        return "<Action: %s>[%s]" % (shortname, data)

class AbstractOneStepActionElement(AbstractActionElement):
    """ One Step Actions are actions which are performed on the fly, they are pushed to the stack by a decision additionally to another element and remove them self from the stack in the same iteration. 
        They allow performing second level tasks like scheduling communication with other robots. 
        The pushing decision element has the ability to push further actions in the same iteration.
    """

    def perform(self, connector, reevaluate=None):
        self.perform_one_step(connector)
        return self.pop()

    def perform_one_step(self, connector, reevaluate=None):
        raise NotImplementedError()
