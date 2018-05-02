import cv2
import rospy
from candidate import Candidate
# NOTE: cv2 drawing functions use (x, y) points!


class DebugImage:
    def __init__(self, image):
        self.raw_image = image.copy()

    def draw_horizon(self, horizon_points, color):
        """
        draws a line where the horizon algorithm found the horizon
        :param horizon_points list of coordinates of the horizon:
        :return void:
        """
        for i in range(len(horizon_points) - 1):
            cv2.line(self.raw_image,
                     horizon_points[i],
                     horizon_points[i+1], color)

    def draw_ball_candidates(self, ball_candidates, color, thickness=1):
        """
        draws a circle around every coordinate where a ball candidate was found
        :param ball_candidates: list of cooordinates of ball candidates of type Candidate
        :param color: color of the circle to draw
        :return void:
        """
        for candidate in ball_candidates:
            cv2.circle(self.raw_image,
                       (candidate.get_center_x(), candidate.get_center_y()),
                       candidate.get_radius(),
                       color,
                       thickness=thickness)

    def draw_points(self, points, color, rad=2):
        for point in points:
            cv2.circle(self.raw_image, point, rad, color)

    def draw_line_segments(self, segments, color, width=2):
        for segment in segments:
            cv2.line(self.raw_image,
                     (segment[0], segment[1]),
                     (segment[2], segment[3]),
                     color)

    def imshow(self):
        """
        Shows the drawn debug image.
        :return void:
        """
        cv2.imshow('Debug Image', self.raw_image)
        cv2.waitKey(1)


class DebugPrinter:
    def __init__(self):
        pass

    @staticmethod
    def print_candidates_info(candidates, name='candidate'):
        if candidates:
            rospy.loginfo('{0} candidates:'.format(len(candidates)))
        else:
            rospy.loginfo('0 candidates.')
        for candidate in candidates:
            DebugPrinter.print_candidate_info(candidate, name=name, prefix='- ')

    @staticmethod
    def print_candidate_info(candidate, name='candidate', prefix=''):
        rospy.loginfo(
            '{0}{1}: x1,y1: {2},{3} | width,height: {4},{5} | rating: {6}'
            .format(
                prefix,
                name,
                candidate.get_upper_left_x(),
                candidate.get_upper_left_y(),
                candidate.get_width(),
                candidate.get_height(),
                candidate.rating))


