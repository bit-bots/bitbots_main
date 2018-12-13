#!/usr/bin/env python

import cv2
import numpy as np

# Generates .png files for map_server, one with lines and one with goalposts
# Default color scheme: black on white background
# Scale: 1 px = 1 cm.

# 2018 rues in centimeter
field_length = 900#547#900
field_width = 600#388#600
goal_width = 150#194#260
goal_area_length = 60#75#100
goal_area_width = 220#290#500
penalty_mark_distance = 130#146#210
center_circle_diameter = 150#120#150
border_strip_width = 70#10#70
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

# Create black image in correct size for goalposts
img_posts = np.zeros(image_size, np.uint8)

# Draw goalposts
img_posts = cv2.circle(img_posts, goalpost_left_1, line_width*2, color, -1)
img_posts = cv2.circle(img_posts, goalpost_left_2, line_width*2, color, -1)
img_posts = cv2.circle(img_posts, goalpost_right_1, line_width*2, color, -1)
img_posts = cv2.circle(img_posts, goalpost_right_2, line_width*2, color, -1)

# Invert images
if invert:
    img_lines = np.ones_like(img_lines) * 255 - img_lines
    img_posts = np.ones_like(img_posts) * 255 - img_posts


# Show image
#cv2.imshow('image', img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

# Save images (to directory where script is executed)
cv2.imwrite('spl.png', img_lines)
#cv2.imwrite('spl.png', img_posts)





