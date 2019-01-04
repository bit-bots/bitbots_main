import numpy as np
import cv2
import rospy
from .color import ColorDetector
from operator import itemgetter
from .debug import DebugPrinter


class HorizonDetector:

    def __init__(self, color_detector, config, debug_printer):
        # type: (np.matrix, ColorDetector, dict, DebugPrinter) -> None
        self._image = None
        self._color_detector = color_detector
        self._horizon_points = None
        self._horizon_full = None
        self._convex_horizon_full = None
        self._horizon_hull = None
        self._debug_printer = debug_printer
        # init config
        self._x_steps = config['horizon_finder_horizontal_steps']
        self._y_steps = config['horizon_finder_vertical_steps']
        self._precise_pixel = config['horizon_finder_precision_pix']
        self._min_precise_pixel = config['horizon_finder_min_precision_pix']

    def set_image(self, image):
        self._image = image
        self._horizon_points = None
        self._horizon_full = None
        self._convex_horizon_full = None
        self._horizon_hull = None

    def get_horizon_points(self):
        # type: () -> list
        """
        calculates the horizon if not calculated yet and returns a list
        containing coordinates on the picture where the horizon is.
        :return list of x,y tuples of the horizon:
        """
        if self._horizon_points is None:
            self._horizon_points = self._sub_horizon()
        return self._horizon_points

    def _sub_horizon(self):
        mask = self._color_detector.mask_image(self._image)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)
        mask = cv2.resize(mask, (self._y_steps, self._y_steps), interpolation=cv2.INTER_LINEAR)
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if mask[y_step, x_step] > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            horizon_points.append((x, firstgreen))
        return horizon_points

    def get_convex_horizon_points(self):
        '''
        returns a set of horizon points that form a convex hull of the
        field
        '''
        if self._horizon_hull is None:
            horizon_points = self.get_horizon_points()

            # 
            # uncomment this block and the one below to view the
            # old and new horizon 
            # (used for the images in the paper)
            #
            # # draw the old horizon line
            # my_img = np.copy(self._image)
            # for i in range(len(horizon_points) - 1):
            #     cv2.line(my_img, horizon_points[i], horizon_points[i+1], (255,0,0))
            # # fill the area below the old horizon black
            # preprocessed_image = np.zeros(self._image.shape)
            # hpoints = np.array([[(0, 0)] +
            #                     horizon_points +
            #                     [(preprocessed_image.shape[1] - 1, 0)]])
            # cv2.fillPoly(preprocessed_image, np.int32(hpoints), 1)

            # calculate the "convex hull" of the horizon points
            horizon_points = self._graham(horizon_points)

            # # fill the area below the new horizon black
            # preprocessed_image2 = np.zeros(self._image.shape)
            # hpoints = np.array([[(0, 0)] +
            #                     horizon_points +
            #                     [(preprocessed_image2.shape[1] - 1, 0)]])
            # cv2.fillPoly(preprocessed_image2, np.int32(hpoints), 1)
            # # show the results
            # cv2.imshow('old horizon', preprocessed_image)
            # cv2.waitKey(1)
            # cv2.imshow('"grahamed" horizon', preprocessed_image2)
            # cv2.waitKey(1)
            # res_image = preprocessed_image2 - preprocessed_image
            # cv2.imshow('areas above the "grahamed" horizon', res_image)
            # cv2.waitKey(1)
            # res_image = preprocessed_image - preprocessed_image2
            # cv2.imshow('potential obstacles', res_image)
            # cv2.waitKey(1)
            # # draw the new horizon line
            # for i in range(len(horizon_points) - 1):
            #     cv2.line(my_img, horizon_points[i], horizon_points[i+1], (0,255,255))
            # cv2.imshow('graham: input blue, output yellow', my_img)
            # cv2.waitKey(1)

            self._horizon_hull = horizon_points

        return self._horizon_hull

    def _graham(self, points):
        '''
        input: list of points (a point is a 2D array (x,y)) with increasing x-coordinates,
               including one point with x = 0 and one point with x = self._image.shape[1]-1
        output: list of points, see below for more detail

        This is a modified Graham's convex hull algorithm. Instead of returning the list
        of points that form the entire convex hull of the input point set, it returns
        only the "half" of the hull which has the lower y-coordinates and spans between the
        points with x=0 and x=self._image.shape[1]-1. 
        '''

        if len(points) < 3:
            # there is no convex hull if less than three points are given
            return points

        # sort by increasing x-coordinates, then sort points with the same x-coordinate
        # by increasing y-coordinates 
        my_points = sorted(points, key=lambda p: (p[0] + 1) * self._image.shape[1] + p[1])

        # take the bottommost point
        p0 = my_points[0]

        # sort the points according to the angle between the vector p0 -> p 
        # and the inverted y-axis
        my_points[1:] = sorted(my_points[1:], key=lambda p: self._graham_point_sort(p, p0))
        num_points = len(my_points)

        # the stack contains the current hull points, top of the stack is the
        # last element in the list
        stack = [my_points[0], my_points[1]]

        #
        # uncomment to show the sorted points
        # (used for the images in the paper)
        #
        # my_img = np.copy(self._image)
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # for j in range(len(my_points)):
        #         cv2.line(my_img, my_points[j], my_points[j], (0,0,255),2)
        #         cv2.putText(my_img,'{}'.format(j),(my_points[j][0], my_points[j][1]),
        #             font, 0.5,(0,255,255),2,cv2.LINE_AA)
        # cv2.imshow("sorted points by angle", my_img)
        # cv2.waitKey(0)

        i = 2
        while (i < num_points) and (stack[-1][0] != self._image.shape[1] - 1):

            #  
            # uncomment to show debug output (perform the algorithm step by step):
            # (used for the images in the paper)
            #
            # cv2.waitKey(10)
            # my_img = np.copy(self._image)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # for j in range(len(stack) - 1):
            #     cv2.line(my_img, stack[j], stack[j+1], (255,255,0))
            #
            # for j in range(len(stack)):
            #     cv2.putText(my_img,'{}'.format(j),stack[j], font, 0.5,(0,255,255),2,cv2.LINE_AA)
            # name = 'graham: i = {}'.format(i)
            #
            # if (len(stack) >= 2):
            #    cv2.putText(my_img,'{}'.format(self._ccw(stack[-1], stack[-2],
            #       my_points[i]) <= 0),(my_points[i][0], my_points[i][1]-20), font,
            #       0.5,(0,255,0),2,cv2.LINE_AA)
            #
            #     cv2.line(my_img, stack[-1], stack[-2], (255,0,0),2)
            # 
            # cv2.line(my_img, my_points[i], my_points[i], (0,0,255),3)
            #
            # cv2.putText(my_img,'|stack| = {}'.format(len(stack)),
            #    (self._image.shape[1]-150, self._image.shape[0]-20), font, 0.5,
            #    (0,0,255),1,cv2.LINE_AA)
            # 
            # cv2.imshow(name, my_img)
            # cv2.waitKey(0)
            # cv2.destroyWindow(name)
            # cv2.waitKey(10)

            if len(stack) < 2 or self._ccw(stack[-1], stack[-2], my_points[i]) <= 0:
                # extend the hull
                stack.append(my_points[i])
                i += 1
            else:
                # an interior angle > 180 degrees is located at the last point in the hull,
                # thus this point cannot be part of the convex hull
                stack.pop()

        return stack

    def _graham_point_sort(self, p, p0):
        '''
        used to sort the points given to Graham's convex hull algorithm
        returns the cosine of the angle between the vector p0->p and the
        inverted y-axis (the vector (0,-1))
        '''
        return -(p0[1] - p[1]) / (np.sqrt((p[0] - p0[0]) ** 2 + (p[1] - p0[1]) ** 2))

    def _ccw(self, p1, p2, p3):
        '''
        returns whether the given points p1, p2 and p3 are 
        counter-clockwise (returns a value > 0)
        clockwise (returns a value < 0) or
        collinear (returns 0) to each other
        '''
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def _mask_horizon(self):
        mask = self._color_detector.mask_image(self._image)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if np.mean(mask[max(0, y - 2):(y + 3),
                           max(0, x - 2):(x + 3)]) > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            horizon_points.append((x, firstgreen))
        return horizon_points


    def _precise_horizon(self):
        # type: () -> list
        """
        Calculates the horizon coordinates in a precise way, but less fast and efficient.
        It checks after having found a horizon if coordinates around this point are also green
        and thus under the horizon.
        It additionally employs checking between the last point known as not the horizon and the horizon point
        to see if the horizon starts somewhere in between. (Currently actually a TODO)
        see also: _fast_horizon()
        :return list of coordinates of the horizon:
        """
        # worst case:
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if self._color_detector.match_pixel(self._image[y][x]):  # when the pixel is in the color space
                    if (y + self._precise_pixel) < min_y:
                        for i in range(self._precise_pixel):
                            greencount = 0
                            if self._color_detector.match_pixel(self._image[y, x]):
                                greencount += 1
                            if greencount >= self._min_precise_pixel:
                                firstgreen = y
                                break
                        firstgreen = y
                        firstgreen_precise = int(round(
                            (firstgreen - y_stepsize)
                            / 2.0))
                        if firstgreen_precise >= 0 and \
                                self._color_detector.match_pixel(
                                    self._image[firstgreen_precise, x]):
                            firstgreen = firstgreen_precise
                        break
            horizon_points.append((x, firstgreen))
        return horizon_points


    def get_full_horizon(self):
        # type: () -> list
        """
        calculates an interpolated list of y coordinates where the horizon is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the horizon is. Index of y value is the x coordinate:
        """
        if self._horizon_full is None:
            xp, fp = zip(*self.get_horizon_points())
            x = list(range(self._image.shape[1]))
            self._horizon_full = np.interp(x, list(xp), list(fp))
        return self._horizon_full

    def get_full_convex_horizon(self):
        # type: () -> list
        """
        calculates an interpolated list of y coordinates where the convex horizon is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the convex horizon is. Index of y value is the x coordinate:
        """
        if self._convex_horizon_full is None:
            xp, fp = zip(*self.get_convex_horizon_points())
            x = list(range(self._image.shape[1]))
            self._convex_horizon_full = np.interp(x, list(xp), list(fp))
        return self._convex_horizon_full


    def candidate_under_horizon(self, candidate, y_offset=0):
        # type: (tuple, int) -> bool
        """
        returns whether the candidate is under the horizon or not
        :param candidate: the candidate, a tuple (upleft_x, upleft_y, width, height)
        :param y_offset: an offset in y-direction (higher offset allows points in a wider range over the horizon)
        :return: whether the candidate is under the horizon or not
        """
        footpoint = (candidate[0] + candidate[2] // 2, candidate[1] + candidate[3] + y_offset)
        return self.point_under_horizon(footpoint)


    def candidates_under_horizon(self, candidates, y_offset=0):
        # type: (list, int) -> list
        return [candidate for candidate in candidates if self.candidate_under_horizon(candidate, y_offset)]


    def balls_under_horizon(self, balls, y_offset=0):
        # type: (list, int) -> list
        return [candidate for candidate in balls if self.candidate_under_horizon(
            (candidate.get_upper_left_x(),
             candidate.get_upper_left_y(),
             candidate.get_width(),
             candidate.get_height()),
            y_offset)]


    def point_under_horizon(self, point, offset=0):
        # type: (tuple, int) -> bool
        """
        returns if given coordinate is a point under horizon
        :param point: coordinate (x, y) to test
        :param offset: offset of pixels to still be accepted as under the horizon. Default is 0.
        :return a boolean if point is under horizon:
        """
        if not 0 <= point[0] < len(self.get_full_horizon()):
            rospy.logwarn('point_under_horizon got called with an out of bounds horizon point')
            return False
        return point[1] + offset > self.get_full_horizon()[point[0]]


    def get_upper_bound(self, y_offset=0):
        # type: () -> int
        """
        returns the y-value of highest point of the horizon (lowest y-value)
        :return: int(), y-value of highest point of the horizon (lowest y-value)
        """
        return max(0, int(min(self.get_horizon_points(), key=itemgetter(1))[1] - y_offset))

    def _equalize_points(self, points):
        # type: (list) -> list
        """
        returns a list of the input points with smoothed y-coordinates to reduce
        the impact of outlier points in the horizon, which are caused by
        detection errors
        :param points: list of input points consisting of tuples (x, y)
        :return: list of input points with smoothed y-coordinates consisting of tuples (x, y)
        """
        equalized_points = list()
        equalized_points.append(points[0])
        buffer0 = points[0]
        buffer1 = points[1]
        for i in range(2, len(points)):
            buffer2 = points[i]
            equalized_points.append((buffer1[0], int(round((((buffer0[1] + buffer2[1]) / 2.0) + buffer1[1]) / 2.0))))
            buffer0 = buffer1
            buffer1 = buffer2
        equalized_points.append(points[-1])
        return equalized_points
