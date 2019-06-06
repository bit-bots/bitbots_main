import cv2
import rospy
from .candidate import Candidate
# NOTE: cv2 drawing functions use (x, y) points!


class DebugImage:
    def __init__(self):
        self.raw_image = None

    def set_image(self, image):
        self.raw_image = image.copy()

    def draw_field_boundary(self, field_boundary_points, color):
        """
        draws a line where the field_boundary algorithm found the field_boundary
        :param field_boundary_points list of coordinates of the field_boundary:
        :return void:
        """
        for i in range(len(field_boundary_points) - 1):
            cv2.line(self.raw_image,
                     field_boundary_points[i],
                     field_boundary_points[i+1], color)

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

    def draw_points(self, points, color, rad=2, thickness=-1):
        for point in points:
            cv2.circle(self.raw_image, point, rad, color, thickness=thickness)

    def draw_line_segments(self, segments, color, width=2):
        for segment in segments:
            cv2.line(self.raw_image,
                     (segment[0], segment[1]),
                     (segment[2], segment[3]),
                     color)

    def get_image(self):
        return self.raw_image

    def imshow(self):
        """
        Shows the drawn debug image.
        :return void:
        """
        cv2.imshow('Debug Image', self.raw_image)
        cv2.waitKey(1)


class DebugPrinter:
    def __init__(self, debug_classes=None):
        self._debug_classes = [debug_class.lower() for debug_class in debug_classes]
        self._all = 'all' in debug_classes

    def set_debug_classes(self, debug_classes):
        self._debug_classes = [debug_class.lower() for debug_class in debug_classes]
        self._all = 'all' in debug_classes
        self.info('reset debug_classes to: ' + str(debug_classes) + '.', 'debug')

    def info(self, message, debug_class=''):
        if self._all or debug_class in self._debug_classes:
            rospy.loginfo(debug_class + ': ' + str(message))

    def warn(self, message, debug_class=''):
        if self._all or debug_class in self._debug_classes:
            rospy.logwarn(debug_class + ': ' + str(message))

    def error(self, message, debug_class=''):
        rospy.logerr(debug_class + ': ' + str(message))

    @staticmethod
    def generate_debug_class_list_from_string(string):
        return string.replace(' ', '').split(',')




