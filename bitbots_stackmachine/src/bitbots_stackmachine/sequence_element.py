# -*- coding:utf-8 -*-
import json
import rospy
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement


class SequenceElement(AbstractDecisionElement):
    """
    This element enables the use of action sequences.
    """

    def __init__(self, connector, init_data):
        """
        init_data has to be a dict like this {"actions": ListOfActions, "action_datas": ListOfInitDatas}
        """
        super(SequenceElement, self).__init__(connector)
        self.actions = init_data["actions"]
        self.action_datas = init_data["action_datas"]
        self.first = True

    
    def perform(self, connector, reevaluate=False):        
        if self.first:
            # push all the actions on the stack
            for i in range(len(self.actions)-1, -1, -1):
                # perform is deactivated to inhibit calling all the actions now
                self.push(self.actions[i], self.action_datas[i], False)
            self.first = False
        else:
            # All actions above us are finished, we can pop
            self.pop()

    def get_reevaluate(self):
        return False

    def __repr__(self):
        """
            Overlaod from the AbstractDecisionElement to have "Sequence" at the start and the actions as data.
        """

        return "<Sequence>[%s]" % (self._init_data)
