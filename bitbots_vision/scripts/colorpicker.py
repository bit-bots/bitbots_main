#!/usr/bin/env python3

import cv2
import numpy as np
import time
import pickle
import os
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Colorpicker(object):
    """
    The bitbots_colorpicker node, which is used to select colors that occur in the field.
    """
    def __init__(self):
        rospy.init_node('bitbots_colorpicker')
        rospy.loginfo('Initializing colorpicker...', logger_name="colorpicker")
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('bitbots_vision')
        self._cv_bridge = CvBridge()
        # Load default image
        self._image = cv2.imread(os.path.join(self._package_path, "img", "Colorpicker.png"))

        # Subscribe to Image-message
        self._sub_image_msg = rospy.Subscriber(
            "image_proc",
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=60000000,
            tcp_nodelay=False)

        # Init
        self._output_usage_info()
        cv2.namedWindow("Colorpicker")
        cv2.setMouseCallback("Colorpicker", self._mouse_callback)
        self._mouse_coord = (0,0)
        self._left_click, self._undo_click = False, False
        self._default_path = "~/.ros/color_space.pickle"
        color_space = np.zeros((256, 256, 256), dtype=np.uint8)

        # Default box size
        self._box_size = 50

        # Init history
        self._history = [color_space]

    def _mask_image(self, image, color_space):
        # type: (np.array, np.array) -> np.array
        """
        Returns the color mask of the image.
        (0 for not in color range and 255 for in color range)
        [Taken from https://github.com/bit-bots/bitbots_vision/blob/master/bitbots_vision/src/bitbots_vision/vision_modules/color.py]

        :param np.array image: input image
        :param np.array color_space: color space
        :return np.array: masked image
        """
        image_reshape = image.reshape(-1,3).transpose()
        mask = color_space[
                image_reshape[0],
                image_reshape[1],
                image_reshape[2],
            ].reshape(
                image.shape[0],
                image.shape[1])
        return mask

    def _mouse_callback(self, event, x, y, flags, param):
        """
        Callback for the OpenCV cursor.

        :param event: Event type
        :param x: Mouse x position
        :param y: Mouse y position
        :param flags: Callback flags
        :param param: Some unused parameter
        """
        self._undo_click = (flags == (cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON))
        self._left_click = (not self._undo_click and event == cv2.EVENT_LBUTTONUP)

        # Set self._mouse_coordinates
        self._mouse_coord = (x, y)

    def _draw_mask(self, image, mask, color, opacity=0.5):
        """
        Draws the mask on an image.

        :param image: The image canvas
        :param mask: The binary mask
        :param color: Color bgr tuple
        :param opacity: Opacity of the mask
        """
        # Make a colored image
        colored_image = np.zeros_like(image)
        colored_image[:, :] = tuple(np.multiply(color, opacity).astype(np.uint8))

        # Compose preview image
        return cv2.add(cv2.bitwise_and(image, image, mask=255-mask),
                cv2.add(colored_image*opacity, image*(1-opacity), mask=mask).astype(np.uint8))

    def _output_usage_info(self):
        """
        Prints some usage information.
        """
        rospy.loginfo(
            "Welcome to the Bit-Bots Colorpicker.\n"
            "Click to select the colors in the OpenCV window.\n"
            "Use <shift> and <left click> to remove colors.\n"
            "To undo a step press 'u'.\n"
            "To increase the size of the selection box press '+', to decrease press '-' \n"
            "To save the color space press 's', then enter the path for the output file in the terminal and confirm with <enter>.\n"
            "Exit using <esc>.\n\n\n", logger_name="colorpicker")

    def _image_callback(self, msg):
        """
        Image message callback.

        :param msg: Image message
        """
        self._image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def ui_loop(self):
        """
        Runs the UI window.

        :return bool: Does the user exit the loop with a save flag set?
        """
        save = False
        while not rospy.is_shutdown():
            # Copy image for the canvas
            canvas = self._image.copy()
            # Get the mouse coordinates
            x, y = self._mouse_coord

            # Get the values for the selection box
            box_min_x = max(int(x - self._box_size / 2), 0)
            box_min_y = max(int(y - self._box_size / 2), 0)
            box_max_x = min(int(x + self._box_size / 2), self._image.shape[1] - 1)
            box_max_y = min(int(y + self._box_size / 2), self._image.shape[0] - 1)

            # Check for click event
            if self._left_click or self._undo_click:
                # Get pixels in the selction
                selected_matrix = self._image[
                                            box_min_y : box_max_y,
                                            box_min_x : box_max_x
                                        ]
                # Serialize the colors
                selected_matrix = selected_matrix.reshape(-1,3)
                # Get the unique colors
                selected_matrix = np.unique(selected_matrix, axis=0)

                # Copy the latest color space
                color_space = self._history[-1].copy()

                # Check for left click or shift + left click
                if self._undo_click:
                    # Remove color values from color space
                    color_space[
                        selected_matrix[:,0],
                        selected_matrix[:,1],
                        selected_matrix[:,2]] = 0
                # Check if we add stuff
                elif self._left_click:
                    # Add color values to color space
                    color_space[
                        selected_matrix[:,0],
                        selected_matrix[:,1],
                        selected_matrix[:,2]] = 1

                # Append new color space to self._history
                self._history.append(color_space)

                # Reset events
                self._left_click = False
                self._undo_click = False

            # Mask the image with the current color space
            mask = self._mask_image(self._image, self._history[-1])

            # Draw the mask on the canvas
            canvas = self._draw_mask(canvas, mask, (255,0,255), opacity=0.8)

            # Draw selection area
            cv2.rectangle(canvas,
                (box_min_x, box_min_y),
                (box_max_x, box_max_y),
                (0, 255, 0),
                2)

            # Show canvas
            cv2.imshow("Colorpicker", canvas)

            # Key checks for UI events
            key = cv2.waitKey(1) & 0xFF

            # Increase selection box size
            if key == ord("+"):
                self._box_size += int(self._box_size * 0.2) + 1
            # Reduce selection box size
            if key == ord("-") and self._box_size > 1:
                self._box_size -= int(self._box_size * 0.2) - 1
            # Undo
            if key == ord("u") and len(self._history) > 1:
                del self._history[-1]
            # Quit exit
            elif key%256 == 27 or 0xFF == ord('q'):
                save = False
                break
            # Save exit
            elif key == ord("s") or key == ord("w"):
                save = True
                break

            time.sleep(0.01)

        # Close all open windows
        cv2.destroyAllWindows()
        return save

    def _save(self):
        """
        Saves the current colorspace in a pickle file.
        """
        # Get user input
        input_path = input( "Press enter to save color space file to '{}' "
                            "or enter output file path: ".format(self._default_path))

        # Check if the user entered something
        if input_path == "":
            # Set to default
            input_path = self._default_path

        # Create absolute path
        input_path = os.path.abspath(os.path.expanduser(input_path))

        # Get the color values
        color_indices = np.where(self._history[-1] == 1)

        # Create data structure for the file
        data = dict(
            red=color_indices[2].tolist(),
            green=color_indices[1].tolist(),
            blue=color_indices[0].tolist()
        )

        # Save
        with open(input_path, 'wb') as outfile:
            pickle.dump(data, outfile, protocol=2)

        rospy.loginfo("Output saved to '{}'.".format(input_path), logger_name="colorpicker")

    def run(self):
        """
        Run the Colorpicker.
        """
        save = self.ui_loop()

        if save:
            self._save()


if __name__ == "__main__":
    colorpicker = Colorpicker()
    colorpicker.run()
