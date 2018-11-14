import json

from bitbots_dsd.abstract_stack_element import AbstractStackElement


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

    def do_not_reevaluate(self):
        """
        Prohibits the next reevaluate.
        This is usefull if you have an action which has to be completed without interruption, e.g. a kick animation.
        """
        self._behaviour.set_do_not_reevaluate()

    def __repr__(self):
        """
            Overlaod from the AbstractStackElement to have "Action" at the start.
        """
        shortname = self.__class__.__name__

        data = json.dumps(self.debug_data)
        self.debug_data = {}

        return "<Action: %s>[%s]" % (shortname, data)


class AbstractOneStepActionElement(AbstractActionElement):
    """ One Step Actions are actions which are performed on the fly, they are pushed to the stack by a decision additionally to another element and remove them self from the stack in the same iteration. 
        They allow performing second level tasks like scheduling communication with other robots. 
        The pushing decision element has the ability to push further actions in the same iteration.
    """

    def perform(self, blackboard, reevaluate=None):
        self.perform_one_step(blackboard)
        return self.pop()

    def perform_one_step(self, blackboard, reevaluate=None):
        raise NotImplementedError()
