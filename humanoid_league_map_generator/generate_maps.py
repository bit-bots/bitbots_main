#!/usr/bin/env python

import cv2
import numpy as np
from scipy import ndimage
import math


# Generates .png files for localization
# Default color scheme: black on white background
# Scale: 1 px = 1 cm.

lines = False
posts = False
fieldboundary = False
features = True

# 2019 field WM
field_length = 900
field_width = 600
goal_width = 260
goal_area_length = 100
goal_area_width = 500
penalty_mark_distance = 150
center_circle_diameter = 150
border_strip_width = 70
line_width = 5

# Invert image image to get black on white background
invert = True

# Choose mark style
# mark_type = 'point'
mark_type = 'cross'

# Line color
color = (255, 255, 255) #white

# Size of complete turf field (field with outside borders)
image_size = (field_width + border_strip_width * 2, field_length + border_strip_width * 2, 3)

# Calculate important points on the field
field_outline_start = (border_strip_width, border_strip_width)
field_outline_end = (field_length + border_strip_width, field_width + border_strip_width)

middle_line_start = (field_length/2 + border_strip_width, border_strip_width)
middle_line_end = (field_length/2 + border_strip_width, field_width + border_strip_width)

middle_point = (field_length/2 + border_strip_width, field_width/2 + border_strip_width)

penalty_mark_left = (penalty_mark_distance + border_strip_width, field_width/2 + border_strip_width)
penalty_mark_right = (image_size[1] - border_strip_width - penalty_mark_distance, field_width/2 + border_strip_width)

goal_area_left_start = (border_strip_width,  border_strip_width + field_width/2 - goal_area_width/2)
goal_area_left_end = (border_strip_width + goal_area_length, field_width/2 + border_strip_width + goal_area_width/2)

goal_area_right_start = (image_size[1] - goal_area_left_start[0], goal_area_left_start[1])
goal_area_right_end = (image_size[1] - goal_area_left_end[0], goal_area_left_end[1])

goalpost_left_1 = (border_strip_width, border_strip_width+field_width/2 + goal_width/2)
goalpost_left_2 = (border_strip_width, border_strip_width+field_width/2 - goal_width/2)

goalpost_right_1 = (image_size[1] - goalpost_left_1[0], goalpost_left_1[1])
goalpost_right_2 = (image_size[1] - goalpost_left_2[0], goalpost_left_2[1])


def drawCross(img, point, width=5, length=10):
    # Might need some fine tuning
    vertical_start = (point[0]-width, point[1] - length)
    vertical_end = (point[0] + width, point[1] + length)
    horizontal_start = (point[0] - length, point[1] - width)
    horizontal_end = (point[0] + length, point[1] + width)
    img = cv2.rectangle(img, vertical_start, vertical_end, color, -1)
    img = cv2.rectangle(img, horizontal_start, horizontal_end, color, -1)

def blurDistance(image, b=5):

	# Calc distances
	distance_map = 255 - ndimage.morphology.distance_transform_edt(255 - image) #todo weniger hin und her rechnen

	# Maximum field distance
	maximum_size = math.sqrt(image.shape[0]**2 + image.shape[1]**2)

	# Get the value to a value from 0 to 1
	distance_map = (distance_map / maximum_size)

	# Magic value please change it
	beta = b

	# Activation function
	distance_map = distance_map ** (2*beta)

	# Scale up
	distance_map = cv2.normalize(distance_map, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

	# To img
	out_img = 100 - distance_map.astype(np.uint8)
	return out_img

def blurGaussian(image):
	out_img = cv2.GaussianBlur(image, (99, 99), cv2.BORDER_DEFAULT)
	out_img = cv2.normalize(out_img, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
	out_img = 100 - out_img
	return out_img


################################################################################
#lines
################################################################################

if lines:

	# Create black image in correct size for lines
	img_lines = np.zeros(image_size, np.uint8)

	# Draw outline
	img_lines = cv2.rectangle(img_lines, field_outline_start, field_outline_end, color, line_width)

	# Draw middle line
	img_lines = cv2.line(img_lines, middle_line_start, middle_line_end, color, line_width)

	# Draw center circle
	img_lines = cv2.circle(img_lines, middle_point, center_circle_diameter / 2, color, line_width)

	# Draw center mark
	if mark_type == 'point':
		img_lines = cv2.circle(img_lines, middle_point, line_width * 2, color, -1)
	else:
		drawCross(img_lines, middle_point)

	# Draw penalty marks
	if mark_type == 'point':
		img_lines = cv2.circle(img_lines, penalty_mark_left, line_width * 2, color, -1)
		img_lines = cv2.circle(img_lines, penalty_mark_right, line_width * 2, color, -1)
	else:
		drawCross(img_lines, penalty_mark_left)
		drawCross(img_lines, penalty_mark_right)

	# Draw goal area
	img_lines = cv2.rectangle(img_lines, goal_area_left_start, goal_area_left_end, color, line_width)
	img_lines = cv2.rectangle(img_lines, goal_area_right_start, goal_area_right_end, color, line_width)

	# blur and write
	cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/lines.png', blurDistance(img_lines, 5))

#############################################################################
#goalposts
#############################################################################

if posts:
	# Create black image in correct size for goalposts
	img_posts = np.zeros(image_size, np.uint8)

	# Draw goalposts
	img_posts = cv2.circle(img_posts, goalpost_left_1, line_width*2, color, -1)
	img_posts = cv2.circle(img_posts, goalpost_left_2, line_width*2, color, -1)
	img_posts = cv2.circle(img_posts, goalpost_right_1, line_width*2, color, -1)
	img_posts = cv2.circle(img_posts, goalpost_right_2, line_width*2, color, -1)
	
	# blur and write
	cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/posts.png', blurGaussian(img_posts))


############################################################################
#field boundary
############################################################################

if fieldboundary:
	# Create black image in correct size (1m padding)
	img_fieldboundary = np.zeros((image_size[0]+200, image_size[1] +200), np.uint8)

	# Draw fieldboundary
	img_fieldboundary = cv2.rectangle(img_fieldboundary, (100, 100), (image_size[1] + 100, image_size[0] + 100), color, line_width)

	# blur and write
	cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/fieldboundary.png', blurDistance(img_fieldboundary, 5)) #TODO oder gaussian?

############################################################################
#features
############################################################################
corners_exact = False
corners_blobs = False
tcrossings = False
tcrossings_blobs = False
crosses_blobs = True

if features:
	size = 30

	############### corners #######################

	if corners_exact:
		# Create black image
		img_corners = np.zeros(image_size, np.uint8)

		#draw outline corners
		#top left
		img_corners = cv2.line(img_corners, field_outline_start,
							   (field_outline_start[0], field_outline_start[1] + size),
							   color, line_width)
		img_corners = cv2.line(img_corners, field_outline_start,
							   (field_outline_start[0] + size, field_outline_start[1]),
							   color, line_width)

		#bottom left
		img_corners = cv2.line(img_corners, (field_outline_start[0], field_outline_end[1]),
							   (field_outline_start[0], field_outline_end[1] - size),
							   color, line_width)
		img_corners = cv2.line(img_corners, (field_outline_start[0], field_outline_end[1]),
							   (field_outline_start[0] + size, field_outline_end[1]),
							   color, line_width)

		#top right
		img_corners = cv2.line(img_corners, (field_outline_end[0], field_outline_start[1]),
							   (field_outline_end[0], field_outline_start[1] + size),
							   color, line_width)
		img_corners = cv2.line(img_corners, (field_outline_end[0], field_outline_start[1]),
							   (field_outline_end[0] - size, field_outline_start[1]),
							   color, line_width)

		#bottom right
		img_corners = cv2.line(img_corners, field_outline_end,
							   (field_outline_end[0], field_outline_end[1] - size),
							   color, line_width)
		img_corners = cv2.line(img_corners, field_outline_end,
							   (field_outline_end[0] - size, field_outline_end[1]),
							   color, line_width)


		# draw left goal area corners
		#top
		img_corners = cv2.line(img_corners, (goal_area_left_end[0], goal_area_left_start[1]),
							   (goal_area_left_end[0], goal_area_left_start[1] + size),
							   color, line_width)
		img_corners = cv2.line(img_corners, (goal_area_left_end[0], goal_area_left_start[1]),
							   (goal_area_left_end[0] - size, goal_area_left_start[1]),
							   color, line_width)

		#bottom


		img_corners = cv2.line(img_corners, goal_area_left_end,
							   (goal_area_left_end[0], goal_area_left_end[1] - size),
							   color, line_width)
		img_corners = cv2.line(img_corners, goal_area_left_end,
							   (goal_area_left_end[0] - size, goal_area_left_end[1]),
							   color, line_width)

		#draw right goal aera corners

		#top

		img_corners = cv2.line(img_corners, (goal_area_right_end[0], goal_area_right_start[1]),
							   (goal_area_right_end[0], goal_area_right_start[1] + size),
							   color, line_width)
		img_corners = cv2.line(img_corners, (goal_area_right_end[0], goal_area_right_start[1]),
							   (goal_area_right_end[0] + size, goal_area_right_start[1]),
							   color, line_width)


		#bottom

		img_corners = cv2.line(img_corners, goal_area_right_end,
							   (goal_area_right_end[0], goal_area_right_end[1] - size),
							   color, line_width)
		img_corners = cv2.line(img_corners, goal_area_right_end,
							   (goal_area_right_end[0] + size, goal_area_right_end[1]),
							   color, line_width)


		# blur and write
		cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/corners.png', blurDistance(img_corners))


	if corners_blobs:

		# Create black image
		img_corners = np.zeros(image_size, np.uint8)

		#field corners
		img_corners = cv2.circle(img_corners, field_outline_start, size, color, -1)
		img_corners = cv2.circle(img_corners, (field_outline_start[0], field_outline_end[1]), size, color, -1)
		img_corners = cv2.circle(img_corners, (field_outline_end[0], field_outline_start[1]), size, color, -1)
		img_corners = cv2.circle(img_corners, field_outline_end, size, color, -1)


		#goal area corners
		img_corners = cv2.circle(img_corners, (goal_area_left_end[0], goal_area_left_start[1]), size, color, -1)
		img_corners = cv2.circle(img_corners, goal_area_left_end, size, color, -1)
		img_corners = cv2.circle(img_corners, (goal_area_right_end[0], goal_area_right_start[1]), size, color, -1)
		img_corners = cv2.circle(img_corners, goal_area_right_end, size, color, -1)

		cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/corners.png', blurGaussian(img_corners))




	############### tcrossings #######################

	if tcrossings:
		# Create black image
		img_tcrossings = np.zeros(image_size, np.uint8)

		#draw left goal area

		#top
		img_tcrossings = cv2.line(img_tcrossings, (goal_area_left_start[0], goal_area_left_start[1]-size/2),
								  (goal_area_left_start[0], goal_area_left_start[1] + size / 2),
								  color, line_width)
		img_tcrossings = cv2.line(img_tcrossings, goal_area_left_start,
								  (goal_area_left_start[0] + size, goal_area_left_start[1]),
								  color, line_width)

		#bottom
		img_tcrossings = cv2.line(img_tcrossings, (goal_area_left_start[0], goal_area_left_end[1] - size / 2),
								  (goal_area_left_start[0], goal_area_left_end[1] + size / 2),
								  color, line_width)
		img_tcrossings = cv2.line(img_tcrossings, (goal_area_left_start[0], goal_area_left_end[1]),
								  (goal_area_left_start[0] + size, goal_area_left_end[1]),
								  color, line_width)

		# blur and write
		cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/tcrossings.png',
					img_tcrossings)

	if tcrossings_blobs:
		# Create black image
		img_tcrossings = np.zeros(image_size, np.uint8)

		#draw blobs for goal areas
		img_tcrossings = cv2.circle(img_tcrossings, goal_area_left_start, size, color, -1)
		img_tcrossings = cv2.circle(img_tcrossings, (goal_area_left_start[0], goal_area_left_end[1]), size, color, -1)
		img_tcrossings = cv2.circle(img_tcrossings, goal_area_right_start, size, color, -1)
		img_tcrossings = cv2.circle(img_tcrossings, (goal_area_right_start[0], goal_area_right_end[1]), size, color, -1)

		# middle line
		img_tcrossings = cv2.circle(img_tcrossings, middle_line_start, size, color, -1)
		img_tcrossings = cv2.circle(img_tcrossings, middle_line_end, size, color, -1)

		# blur and write
		cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/tcrossings.png',
					blurGaussian(img_tcrossings))



	############### crosses #######################

	if crosses_blobs:
		# Create black image
		img_crosses = np.zeros(image_size, np.uint8)

		# penalty marks
		img_crosses = cv2.circle(img_crosses, penalty_mark_left, size, color, -1)
		img_crosses = cv2.circle(img_crosses, penalty_mark_right, size, color, -1)

		# middle point
		img_crosses = cv2.circle(img_crosses, middle_point, size, color, -1)

		# center circle middle line crossings
		img_crosses = cv2.circle(img_crosses, (middle_point[0], middle_point[1] - center_circle_diameter), size, color, -1)
		img_crosses = cv2.circle(img_crosses, (middle_point[0], middle_point[1] + center_circle_diameter), size, color, -1)

		# blur and write
		cv2.imwrite('/home/judith/robocup/bitbots_meta/humanoid_league_localization/models/crosses.png',
					blurGaussian(img_crosses))






