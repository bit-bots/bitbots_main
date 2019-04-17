#!/usr/bin/env python2



class VisualCompass:
    """
    Interface for an Visual compass, that gets new images and calculates an horizontal angle with a corresponding
    confidence.
    """
    def process_image(self, image, resultCB=None, debugCB=None):
        # type: (np.array, func, func) -> None
        """
        Processes an image and updates the internal state
        Calls Callbacks with updated state if callback is supplied

        :param image: 2D numpy Array with RGB channels
        :param resultCB: Callback called with updated state (angle: float [0:2*pi], confidence: float [0,1])
        :param debugCB: Callback called with debug image (image: numpy Array)
        :return: void
        """
        pass

    def set_config(self, config):
        # type: (dict) -> None
        """
        Updates the configuration
        config content TBD

        :param config: Dictionary with new config
        :return: void
        """
        pass

    def set_truth(self, angle, image):
        # type: (float, np.array) -> None
        """
        set a truth-image for specific angle.

        :param angle: float [0:2*pi]
        :param image: 2D numpy Array
        :return: void
        """
        pass

    def get_side(self):
        # type: () -> (float, float)
        """
        :return: current internal state (angle: [0:2*pi], confidence: [0,1]) May return (None,None).
        """
        pass


