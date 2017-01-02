# -*- coding:utf-8 -*-
"""
AnimationCapsule
^^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/4/14: Created (sheepy)

"""


class AnimationCapsule:
    def __init__(self, data):
        self.data = data

    def play_animation(self, ani):
        """
        plays the animation "ani" and sets the flag "BusyAnimation"

        :param ani: name of the animation which shall be played
        :type ani: str
        :return bool: Could he give the animation to the animation module
        """
        if self.data.get("BusyAnimation", False):
            # is currently busy, so can't start a new animation
            return False
        else:
            self.data["BusyAnimation"] = True
            self.data["Animation"] = (ani, self.finished_animation)
            return True

    def get_current_animation(self):
        if self.data.get("Animation", None) is not None:
            return self.data["Animation"][0]
        else:
            return "None"

    def finished_animation(self, args):  # todo args?
        """
        Resets the flag after beeing given to the animation framework in play_animation(ani)
        has been called after an animation
        """
        self.data["BusyAnimation"] = False

    def is_animation_busy(self):
        """
        Checks if an animation is currently played
        """
        return self.data.get("BusyAnimation", False)
