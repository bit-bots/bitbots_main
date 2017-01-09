from bitbots_common.stackmachine.abstract import AbstractStackElement


class AbstractDecisionModule(AbstractStackElement):
    def __repr__(self):
        """
        Wir kürzen die Repräsentation ab, ist so kürzer, und sagt
        trotzdem noch genug
        """
        return "<Decision: " + \
               self.__class__.__module__.split('.')[-2] \
               + "." + self.__class__.__name__ + ">"
