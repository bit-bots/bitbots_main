import cv2
import rospy
from .candidate import Candidate
# NOTE: cv2 drawing functions use (x, y) points!


class DebugImage:
    def __init__(self):
        self.raw_image = None

    def set_image(self, image):
        self.raw_image = image.copy()

    def draw_field_boundary(self, field_boundary_points, color, thickness=1):
        """
        draws a line where the field_boundary algorithm found the field_boundary
        :param field_boundary_points list of coordinates of the field_boundary:
        :return void:
        """
        for i in range(len(field_boundary_points) - 1):
            cv2.line(self.raw_image,
                     field_boundary_points[i],
                     field_boundary_points[i+1], color, thickness=1)

    def draw_ball_candidates(self, ball_candidates, color, thickness=1):
        """
        draws a circle around every coordinate where a ball candidate was found
        :param ball_candidates: list of cooordinates of ball candidates of type Candidate
        :param color: color of the circle to draw
        :return void:
        """
        for candidate in ball_candidates:
            if candidate:
                cv2.circle(self.raw_image,
                           (candidate.get_center_x(), candidate.get_center_y()),
                           candidate.get_radius(),
                           color,
                           thickness=thickness)

    def draw_obstacle_candidates(self, obstacle_candidates, color, thickness=1):
        for candidate in obstacle_candidates:
            if candidate:
                cv2.rectangle(self.raw_image,
                              candidate.get_upper_left_point(),
                              candidate.get_lower_right_point(),
                              color,
                              thickness=thickness)

    def draw_points(self, points, color, thickness=-1, rad=2):
        for point in points:
            cv2.circle(self.raw_image, point, rad, color, thickness=thickness)

    def draw_line_segments(self, segments, color, thickness=2):
        for segment in segments:
            cv2.line(self.raw_image,
                     (segment[0], segment[1]),
                     (segment[2], segment[3]),
                     color, thickness=2)

    def get_image(self):
        return self.raw_image

    def draw(self, debug_image_description, image=None):
        """
        Draws a debug image description, that contains the style and the date for each object/class that we debug
        :param debug_image_description: List of dicts contains the style and the date for each object/class that we debug
        In the dict 'type' refers to the type that we want to draw. Some types are ['obstacle', 'field_boundary', 'ball', 'line_point', 'line_segment'].
        The key 'color' defines the color as BRG. For most types this is the border color.
        The key 'thickness' refers to the border thickness.
        The data, so the candidates we want to draw are defined with the 'data' key.
        :return: Image with debug stuff
        """
        # Set Image if transmitted (optional). Otherwise take the manual set image.
        if image:
            self.set_image(image)
        # Define the draw functions for each type
        draw_functions = {
            'obstacle' : self.draw_obstacle_candidates,
            'field_boundary': self.draw_field_boundary,
            'ball' : self.draw_ball_candidates,
            'line_point' : self.draw_points,
            'line_segment' : self.draw_line_segments,
        }
        # Draw all entries
        for draw_type in debug_image_description:
            # Get drawing function from dict
            draw_function = draw_functions[draw_type['type']]
            # Call drawing function
            draw_function(draw_type['data'], draw_type['color'], draw_type['thickness'])
        # Return the image
        return self.get_image()

    def imshow(self):
        """
        Shows the drawn debug image.
        :return void:
        """
        cv2.imshow('Debug Image', self.raw_image)
        cv2.waitKey(1)
