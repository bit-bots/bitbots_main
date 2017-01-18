from stackmachine.abstract_stack_element import AbstractStackElement


class AbstractActionModule(AbstractStackElement):
    def __repr__(self):
        """
        Wir kürzen die Repräsentation ab, ist so kürzer, und sagt
        trotzdem noch genug
        """
        return "<Action: " + \
               self.__class__.__module__.split('.')[-2] \
               + "." + self.__class__.__name__ + ">"
