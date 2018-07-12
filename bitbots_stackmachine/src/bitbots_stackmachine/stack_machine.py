# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from bitbots_stackmachine.abstract_stack_element import AbstractStackElement


class StackMachine(object):
    """
    One decision is defined as the root decision, the starting point.
    Each decision element, which is pushed on the stack, is immediately executed until no further element is pushed. 
    Following, each iteration, for each element, is checked if it requires to be reevaluated and finally the top element of the stack will be executed, usually an action.
    If the outcome of a reevaluated element changes, the entire stack on top of this element will be dropped and the stack newly constructed.
    As soon as the action is complete, the element will be popped off the stack and the module underneath will be executed in the next iteration.
    If this is a decision, it again pushes a further decision or an action and the new top element will be executed.

    By this structure, it is always visible which action the robot tries to perform and which decisions were made.

    If a new element is pushed on top of the stack, it is directly executed.
    In most cases, the pushing element is completing its execution with the push of another element. 
    Any following code will be executed as soon as the stack is not further expanded.
    """

    stack = []
    start_element = None
    start_element_data = None
    stack_excec_index = -1
    stack_reevaluate = False
    do_not_reevaluate = False
    old_representation = ""

    def __init__(self, connector, debug_topic):
        self.connector = connector

        self.debug_active = rospy.get_param("debug_active", False)
        if self.debug_active:
            self.debug_pub = rospy.Publisher(debug_topic, String, queue_size=100)

    def _init_element(self, element, init_data=None):
        """
            Initialises the element.
        """
        mod = element(self.connector, init_data)
        mod.setup_internals(self, init_data)
        return mod

    def set_start_element(self, start_element, init_data=None):
        """
            This method dfines the start element on the stack, which stays always on the bottom of the stack.

            This method should be called in __init__.
        """
        self.stack = []
        self.start_element = start_element
        self.start_element_data = init_data
        self.stack.append(self._init_element(start_element, init_data))

    def interrupt(self):
        """
            An interrupt is an event which clears the complete stack to reset the behavior.
            In the special case of RoboCup, we use it when the game-state changes, but it can also be used for example if the robot is kidnapped or paused.
            In the following iteration, the stack will be newly created starting at the root element.
        """
        if self.stack_reevaluate:
            # we were currently checking preconditions 
            # we stop this, so that update() knows that it has to stop
            self.stack_reevaluate = False        
        self.stack = [self._init_element(self.start_element,
                                        self.start_element_data)]

    def update(self, reevaluate=True):
        """
        Calls the element which is currently on top of the stack.
        Before doing this, all preconditions are checked (all decision elements where reevaluate is true).
        
        :param: reevaluate: Can be set to False to inhibit the reevaluation
        :type reevaluate: bool
        """
        self.publish_debug_msg()

        if reevaluate and not self.do_not_reevaluate:
            self.stack_excec_index = 0
            self.stack_reevaluate = True
            for element in self.stack[:-1]:
                # check all elements, exept the top one
                if element.get_reevaluate():
                    element.perform(self.connector, True)
                    if not self.stack_reevaluate:
                        # We had some external interrupt, we stop here
                        return
                self.stack_excec_index += 1
            self.stack_reevaluate = False        
        if reevaluate:
            # reset flag
            self.do_not_reevaluate = False
        # run the top module
        self.stack[-1].perform(self.connector)
  
    def push(self, element, init_data=None):
        """
        Put a new element on the stack and start it directly. 

        .. warning::
            After using push, you should not have further code, since this
            leads to difficult to debug behavior. Try to use::

                return self.push(xxxElement, data)            

        :param element: The element that should be put on top of the stack. Do not initilize!            
        :type element: Element
        :param init_data: This data will be given to the new module during its init, optional
        """
        if self.stack_reevaluate:
            # we are currently checking pre conditions
            # check if we made the same decision (push) as last time
            if type(self.stack[self.stack_excec_index + 1]) == element and \
                            self.stack[self.stack_excec_index + 1].get_init_data() \
                            == init_data:
                # decision was the same, reevaluation passed, precondition did not change
                return
            else:
                # result changed. we clear all stack above us and push the new element on top
                self.stack = self.stack[0:self.stack_excec_index + 1]
                # reevaluate is finished
                self.stack_reevaluate = False
        self.stack.append(self._init_element(element, init_data))
        # we call the new element without another reevaluate
        self.update(False)

    def pop(self):
        """
        Removes the element from the stack. The previous element will not be called again.
        """
        if len(self.stack) > 1:
            if self.stack_reevaluate:
                # we are currently reevaluating. we shorten the stack here                
                if self.stack_excec_index > 0:
                    # only shorten stack if it still has one element
                    self.stack = self.stack[0:self.stack_excec_index]
                # stop reevaluating
                self.stack_reevaluate = False
            else:
                self.stack.pop()
        else:
            rospy.logwarn("Can't pop in %s: stack is empty, Module: %s" % (repr(self), repr(self.stack[0])))

    def set_do_not_reevaluate(self):
        """No reevaluation on next iteration"""
        self.do_not_reevaluate = True

    def get_stack(self):
        """
        Returns the current stack
        """
        return self.stack
    
    def publish_debug_msg(self):
        """
        Helper method to publish debug data
        """
        if self.debug_active:
            msg_data = ",".join([repr(x) for x in self.stack])
            msg = String(data=msg_data)
            self.debug_pub.publish(msg)
