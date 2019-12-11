#!/usr/bin/env python3

import cv2
import numpy as np
import time
import pickle
import os
import rospy
import VisionExtensions
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node('bitbots_colorpicker')
rospy.loginfo('Initializing colorpicker...', logger_name="colorpicker")
_cv_bridge = CvBridge()


def click_and_crop(event, x, y, flags, param):
	"""
	Callback for the OpenCV cursor.

	:param event: Event type
	:param x: Mouse x position
	:param y: Mouse y position
	:param flags: Callback flags
	:param param: Some unused parameter
	"""
	global coord, left_click, undo_click

	if flags == (cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON):
		# Substracts colors
		undo_click = True
	elif event == cv2.EVENT_LBUTTONUP:
		# Adds colors
		left_click = True

	# Set coordinates
	coord = (x, y)


def draw_mask(image, mask, color, opacity=0.5):
	"""
	Draws the mask on an image

	:param image: The image canvas
	:param mask: The binary mask
	:param color: Color bgr tuple
	:param opacity: Opacity of the mask
	"""
	# Make a colored image
	colored_image = np.zeros_like(image)
	colored_image[:, :] = tuple(np.multiply(color, opacity).astype(np.uint8))

	# Compose debug image with lines
	return cv2.add(cv2.bitwise_and(image, image, mask=255-mask),
			cv2.add(colored_image*opacity, image*(1-opacity), mask=mask).astype(np.uint8))


def init_text():
	"""
	Prints some usage information.
	"""
	rospy.loginfo( "Welcome to the Colorpicker\n\
Click to select the colors in the OpenCV window.\n\
Use <shift> and <left click> to remove colors.\n\
To undo a step press 'u'.\n\
To save the color space press <s> followed by the path and <enter>.\n\
Exit using <esc>.\n\n\n", logger_name="colorpicker")

def image_callback(msg):
	"""
	Image message callback

	:param msg: Image message
	"""
	global image
	image = _cv_bridge.imgmsg_to_cv2(msg, 'bgr8')


image = np.zeros((300,300,3), dtype=np.uint8)

# Subscribe to Image-message
sub_image_msg = rospy.Subscriber(
	"image_raw",
	Image,
	image_callback,
	queue_size=1,
	buff_size=60000000,
	tcp_nodelay=False)

# Init
init_text()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
coord = (0,0)
left_click, undo_click = False, False
color_space = np.zeros((256, 256, 256), dtype=np.uint8)

# Default box size
box_size = 50

# Init history
history = [color_space]

while not rospy.is_shutdown():
	# Copy image for the canvas
	canvas = image.copy()
	# Get the mouse coordinates
	x, y = coord

	# Get the values for the selection box
	box_min_x = max(int(x - box_size / 2), 0)
	box_min_y = max(int(y - box_size / 2), 0)
	box_max_x = min(int(x + box_size / 2), image.shape[1] - 1)
	box_max_y = min(int(y + box_size / 2), image.shape[0] - 1)

	# Check for click event
	if left_click or undo_click:
		# Get pixels in the selction
		selected_matrix = image[
									box_min_y : box_max_y,
									box_min_x : box_max_x
								]
		# Serialize the colors
		selected_matrix = selected_matrix.reshape(-1,3)
		# Get the unique colors
		selected_matrix = np.unique(selected_matrix, axis=0)

		# Copy the latest color space
		color_space = history[-1].copy()

		# Check if we do or or undo
		if undo_click:
			# Remove values in color space
			color_space[
				selected_matrix[:,0],
				selected_matrix[:,1],
				selected_matrix[:,2]] = 0
		# Check if we add stuff
		elif left_click:
			# Set values for colors in color space
			color_space[
				selected_matrix[:,0],
				selected_matrix[:,1],
				selected_matrix[:,2]] = 1
		else:
			print("This should never happen")

		# Append new color space to history
		history.append(color_space)

		# Reset events
		left_click, undo_click = False, False

	# Mask the image with the current color space
	mask = VisionExtensions.maskImg(image, history[-1])

	# Draw the mask on the canvas
	canvas = draw_mask(canvas, mask, (255,0,255), opacity=0.8)

	# Draw selection area
	cv2.rectangle(canvas,
		(
			box_min_x,
			box_min_y
		), (
			box_max_x,
			box_max_y,),
		(0, 255, 0), 2)

	# Show canvas
	cv2.imshow("image", canvas)


	# Key checks for UI events
	key = cv2.waitKey(1) & 0xFF

	# Increase selection size
	if key == ord("+"):
		box_size += int(box_size * 0.2)
	# Reduce selection size
	if key == ord("-") and box_size > 1:
		box_size -= int(box_size * 0.2)
	# Undo
	if key == ord("u") and len(history) > 1:
		del history[-1]
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

if save:
	# Get user input
	input_path = input("Enter color space path: ")

	# Check if the user entered something
	if input_path == "":
		# Set to default
		input_path = "~/.ros/color_space.pickle"

	# Create abs path
	input_path = os.path.abspath(os.path.expanduser(input_path))

	# Get the color values
	color_indices = np.where(history[-1] == 1)

	# Create data struckture for the file
	data = dict(
		red=color_indices[0].tolist(),
		green=color_indices[1].tolist(),
		blue=color_indices[2].tolist()
	)

	# Save
	with open(input_path, 'wb') as outfile:
		pickle.dump(data, outfile, protocol=2)

	rospy.loginfo("Output saved to '{}'.".format(input_path), logger_name="colorpicker")

