import numpy as np
import cv2
import rospy
from sensor_msgs.msg import JointState
from .color import ColorDetector
from operator import itemgetter
from .debug import DebugPrinter
from .evaluator import RuntimeEvaluator


class FieldBoundaryDetector:

    def __init__(self, field_color_detector, config, debug_printer, runtime_evaluator=None, used_by_dyn_color_detector=False):
        # type: (ColorDetector, dict, DebugPrinter, RuntimeEvaluator, bool) -> None
        """
        This module can compute different versions of the field boundary.
        :param field_color_detector: checks whether a color is part of the field colors
        :param config: the configuration contained in visionparams.yaml
        :param debug_printer: outputs debug messages, necessary only for debug
        :param runtime_evaluator: can be used to compute runtime of methods
        :param used_by_dyn_color_detector: True when FieldBoundaryDetector is used by DynamicColorSpace
        """
        # set variables:
        self._image = None
        self._head_joint_position = None
        self._field_boundary_points = None
        self._field_boundary_full = None
        self._convex_field_boundary_points = None
        self._convex_field_boundary_full = None
        self._mask = None
        self._field_color_detector = field_color_detector
        self._debug_printer = debug_printer
        self._runtime_evaluator = runtime_evaluator
        self._used_by_dyn_color_detector = used_by_dyn_color_detector
        # init config:
        self._x_steps = config['field_boundary_finder_horizontal_steps']
        self._y_steps = config['field_boundary_finder_vertical_steps']
        self._roi_height = config['field_boundary_finder_roi_height']
        self._roi_width = config['field_boundary_finder_roi_width']
        self._roi_increase = config['field_boundary_finder_roi_increase']
        self._green_threshold = config['field_boundary_finder_green_threshold']
        self._precise_pixel = config['field_boundary_finder_precision_pix']
        self._min_precise_pixel = config['field_boundary_finder_min_precision_pix']
        self._head_joint_threshold = config['field_boundary_finder_head_joint_threshold']
        # changes the search method when the FieldBoundaryDetector is used by the dynamic colorspace
        if used_by_dyn_color_detector:
            self._search_method = config['dynamic_color_space_field_boundary_finder_search_method']
        else:
            self._search_method = config['field_boundary_finder_search_method']

    def set_image(self, image):
        # type: (np.matrix) -> None
        """
        refreshes the variables after receiving an image
        :param image: the current frame of the video feed
        """
        self._image = image
        self._field_boundary_points = None
        self._field_boundary_full = None
        self._convex_field_boundary_full = None
        self._convex_field_boundary_points = None
        self._mask = None

    def set_head_joint_state(self, head_joint_states):
        # type: (JointState) -> None
        """
        refreshes the vertical tilt of the head
        :param head_joint_states: the current tilts of the head
        """
        vertical_head_tilt = head_joint_states.name.index('HeadTilt')
        self._head_joint_position = head_joint_states.position[vertical_head_tilt]

    def get_mask(self):
        # type: () -> np.array
        """
        :return: np.array
        """
        if self._mask is None:
            self._mask = self._compute_mask()
        return self._mask

    def _compute_mask(self):
        # type: () -> np.array
        """
        calculates a mask that contains white pixels below the field-boundary
        :return: np.array
        """
        shape = np.shape(self._image)
        img_size = (shape[0], shape[1])
        # Generates a white canvas
        canvas = np.ones(img_size, dtype=np.uint8) * 255
        hpoints = np.array([[(0, 0)] + self.get_field_boundary_points() + [(shape[1] - 1, 0)]])
        # Blacks out the part over the field_boundary
        return cv2.fillPoly(canvas, hpoints, 0)

    def get_field_boundary_points(self, offset=0):
        # type: (int) -> list
        """
        calculates the field-boundary if not calculated yet and returns a list
        containing coordinates on the picture where the field-boundary is.
        the offset works UPWARDS!
        :return list of x,y tuples of the field_boundary:
        """
        if self._field_boundary_points is None:
            self._compute_field_boundary_points()
        # applying the offset
        if offset != 0:
            return [(point[0], point[1] - offset) for point in self._field_boundary_points]
        return self._field_boundary_points

    def _compute_field_boundary_points(self):
        # type:
        """
        calls the method specified in the config (visionparams.yaml) to compute the field boundary points
        :return: points of the field_boundary as a list of x,y tuples
        """
        self._field_boundary_points = []
        if self._search_method == 'dynamic':
            # decides the search method depending on the vertical tilt of the head
            if self._head_joint_position and self._head_joint_position > self._head_joint_threshold:
                self._field_boundary_points = self._sub_field_boundary_points_reversed()
            else:
                self._field_boundary_points = self._sub_field_boundary_points_iteration()
        elif self._search_method == 'binary':
            self._field_boundary_points = self._sub_field_boundary_points_binary()
        elif self._search_method == 'reversed':
            self._field_boundary_points = self._sub_field_boundary_points_reversed()
        else:
            # default search method:
            self._field_boundary_points = self._sub_field_boundary_points_iteration()

    def _sub_field_boundary_points_binary(self):
        # type: () -> list
        """
        Finds the points of the field edge visible in the image. Uses a faster binary search method, that unfortunately
        finds these points below field lines sometimes
        :return: points of the field_boundary as a list of x,y tuples
        """
        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = self._field_color_detector.mask_image(self._image)
        # noise reduction on the field_mask:
        field_mask = cv2.morphologyEx(
            field_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)

        # the stepsize is the number of pixels traversed in the image by going one step
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)

        # the region of interest (roi) for a specific point is a rectangle
        # with the point in the middle of its top row; the point is slightly left of center when the width is even
        roi_start_height_y = self._roi_height
        roi_start_width_x = self._roi_width
        roi_start_radius_x = roi_start_width_x // 2
        # increase of roi radius per pixel, e.g. 0.1 increases the radius by 1 for every 10 pixels
        # this accommodates for size difference of objects in the image depending on their distance
        # and therefore height in image:
        roi_increase = self._roi_increase
        # height/width/radius-in-x-direction of the roi after maximum increase at the bottom of the image
        roi_max_height_y = roi_start_height_y + int(self._image.shape[0] * roi_increase * 2)
        roi_max_width_x = roi_start_width_x + int(self._image.shape[0] * roi_increase * 2)
        roi_max_radius_x = roi_max_width_x // 2
        # extents the outermost pixels of the image as the roi will go beyond the image at the edges
        # Syntax: cv2.copyMakeBorder(image, top, bottom, left, right, type of extension)
        field_mask = cv2.copyMakeBorder(field_mask, 0, roi_max_height_y, roi_max_radius_x, roi_max_radius_x,
                                        cv2.BORDER_REPLICATE)

        # uncomment this to use a kernel for the roi
        # kernel = np.zeros((roi_height, roi_width))  # creates a kernel with 0 everywhere
        # kernel[int(kernel_height/2):, :] = 1  # use this to fill in other values at specific places in the kernel

        green_threshold = self._green_threshold
        for x_step in range(0, self._x_steps):  # traverse columns (x_steps)
            roi_mean = 0
            y_step = 0
            roi_current_height_y = 0
            # binary search for finding the first green pixel:
            first = 0  # top of the column
            last = self._y_steps - 1  # bottom of the column
            # calculate the x coordinate in the image of a column:
            x_image = int(round(x_step * x_stepsize)) + roi_max_radius_x
            while first < last:
                y_step = (first + last) // 2
                y_image = int(round(y_step * y_stepsize))  # calculate the y coordinate in the image of a y_step

                # creates the roi for a point (y_image, x_image)
                roi_current_radius_x = roi_start_radius_x + int(y_image * roi_increase)
                roi_current_height_y = roi_start_height_y + int(y_image * roi_increase * 2)
                roi = field_mask[y_image:y_image + roi_current_height_y,
                                 x_image - (roi_current_radius_x - 1):x_image + roi_current_radius_x]

                roi_mean = roi.mean()
                # roi_mean = (roi * kernel).sum()  # uncomment when using a kernel
                if roi_mean > green_threshold:  # is the roi green enough?
                    # the value is green enough, therefore the field_boundary is somewhere above this point
                    # the area left to search can be halved by setting the new "last" above "y_current"
                    last = y_step - 1
                else:
                    # the value isn't green enough, therefore the field_boundary is somewhere below this point
                    # the area left to search can be halved by setting the new "first" below "y_current"
                    first = y_step + 1
            # During binary search the field_boundary is either approached from the bottom or from the top.
            # When approaching the field_boundary from the top,
            # y_step stops one step above the field_boundary on a non green point.
            # Therefore the y_step has to be increased when it stops on a non green point.
            if roi_mean <= green_threshold:
                y_step += 1
            # calculate the y coordinate in the image of the y_step the topmost green pixel was found on with an offset
            # of roi_height, as the region of interest extends this far below the point
            y_image = int(round(y_step * y_stepsize)) + roi_current_height_y
            self._field_boundary_points.append((x_image - roi_max_radius_x, y_image))
        return self._field_boundary_points

    def _sub_field_boundary_points_reversed(self):
        # type: () -> list
        """
        Finds the points of the field boundary visible in the image. Uses the reversed method iterating from bottom to
        top until it finds enough non green points. Useful for when two fields are adjacent to each other.
        :return: points of the field_boundary as a list of x,y tuples
        """

        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = self._field_color_detector.mask_image(self._image)
        # noise reduction on the field_mask:
        field_mask = cv2.morphologyEx(
            field_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)

        # the stepsize is the number of pixels traversed in the image by going one step
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)

        # the region of interest (roi) for a specific point is a rectangle with the point in the middle of its top row
        # the point is slightly left of center when the width is even
        roi_start_height_y = self._roi_height
        roi_start_width_x = self._roi_width
        roi_start_radius_x = roi_start_width_x // 2
        # increase of roi radius per pixel, e.g. 0.1 increases the radius by 1 for every 10 pixels
        # this accommodates for size difference of objects in the image depending on their distance
        # and therefore height in image:
        roi_increase = self._roi_increase
        # height/width/radius-in-x-direction of the roi after maximum increase at the bottom of the image
        roi_max_height_y = roi_start_height_y + int(self._image.shape[0] * roi_increase * 2)
        roi_max_width_x = roi_start_width_x + int(self._image.shape[0] * roi_increase * 2)
        roi_max_radius_x = roi_max_width_x // 2
        # extents the outermost pixels of the image as the roi will go beyond the image at the edges
        # Syntax: cv2.copyMakeBorder(image, top, bottom, left, right, type of extension)
        field_mask = cv2.copyMakeBorder(field_mask, roi_start_height_y, 0, roi_max_radius_x, roi_max_radius_x,
                                        cv2.BORDER_REPLICATE)

        # uncomment this to use a kernel for the roi
        kernel = np.ones((roi_max_height_y, roi_max_width_x))  # creates a kernel with 0 everywhere
        # use this to fill in other values at specific places in the kernel
        kernel[0: int(roi_max_height_y // 5), int(roi_max_width_x // 2.2): int(roi_max_width_x - roi_max_width_x // 2.2)] = 10

        green_threshold = self._green_threshold
        for x_step in range(self._x_steps):  # traverse columns (x_steps)
            top_green = roi_start_height_y  # set field_boundary point to worst case
            # calculate the x coordinate in the image of a column:
            x_image = int(round(x_step * x_stepsize)) + roi_max_radius_x
            for y_step in range(self._y_steps):
                # calculate the y coordinate in the image of a y_step:
                y_image = int(self._image.shape[0] - (round(y_step * y_stepsize))) + roi_start_height_y

                # creates the roi for a point (y_image, x_image)
                roi_current_radius_x = roi_start_radius_x + int(y_image * roi_increase)
                roi_current_height_y = roi_start_height_y + int(y_image * roi_increase * 2)
                roi = field_mask[y_image - roi_current_height_y:y_image,
                                 x_image - (roi_current_radius_x - 1):x_image + roi_current_radius_x]

                # roi_mean = roi.mean()

                roi_mean = (roi * kernel[roi_current_height_y - 1, roi_current_radius_x * 2 - 1]).mean()  # uncomment when using a kernel
                if roi_mean <= green_threshold:
                    top_green = y_image
                    break
            self._field_boundary_points.append((x_image - roi_max_radius_x, top_green))
        return self._field_boundary_points

    def _sub_field_boundary_points_iteration(self):
        # type: () -> list
        """
        Finds the points of the field boundary visible in the image. Uses the standard method iterating from top to
        bottom until it finds enough green points.
        :return: points of the field_boundary as a list of x,y tuples
        """

        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = self._field_color_detector.mask_image(self._image)
        # noise reduction on the field_mask:
        field_mask = cv2.morphologyEx(
            field_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)

        # Syntax: cv2.resize(image, (width, height), type of interpolation)
        field_mask = cv2.resize(field_mask, (self._x_steps, self._y_steps), interpolation=cv2.INTER_LINEAR)

        # the stepsize is the number of pixels traversed in the image by going one step
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)

        min_y = self._image.shape[0] - 1
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set field_boundary point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if field_mask[y_step, x_step] > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            self._field_boundary_points.append((x, firstgreen))
        return self._field_boundary_points

    def get_convex_field_boundary_points(self):
        '''
        returns a set of field_boundary points that form a convex hull of the
        field
        '''
        if self._convex_field_boundary_points is None:
            self._compute_convex_field_boundary_points()
        return self._convex_field_boundary_points

    def _compute_convex_field_boundary_points(self):
        """
        returns a set of field_boundary points that form a convex hull of the
        field
        """
        field_boundary_points = self.get_field_boundary_points()

        #
        # uncomment this block and the one below to view the
        # old and new field_boundary
        # (used for the images in the paper)
        #
        # # draw the old field_boundary line
        # my_img = np.copy(self._image)
        # for i in range(len(field_boundary_points) - 1):
        #     cv2.line(my_img, field_boundary_points[i], field_boundary_points[i+1], (255,0,0))
        # # fill the area below the old field_boundary black
        # preprocessed_image = np.zeros(self._image.shape)
        # hpoints = np.array([[(0, 0)] +
        #                     field_boundary_points +
        #                     [(preprocessed_image.shape[1] - 1, 0)]])
        # cv2.fillPoly(preprocessed_image, np.int32(hpoints), 1)

        # calculate the "convex hull" of the field_boundary points
        field_boundary_points = self._graham(field_boundary_points)

        # # fill the area below the new field_boundary black
        # preprocessed_image2 = np.zeros(self._image.shape)
        # hpoints = np.array([[(0, 0)] +
        #                     field_boundary_points +
        #                     [(preprocessed_image2.shape[1] - 1, 0)]])
        # cv2.fillPoly(preprocessed_image2, np.int32(hpoints), 1)
        # # show the results
        # cv2.imshow('old field_boundary', preprocessed_image)
        # cv2.waitKey(1)
        # cv2.imshow('"grahamed" field_boundary', preprocessed_image2)
        # cv2.waitKey(1)
        # res_image = preprocessed_image2 - preprocessed_image
        # cv2.imshow('areas above the "grahamed" field_boundary', res_image)
        # cv2.waitKey(1)
        # res_image = preprocessed_image - preprocessed_image2
        # cv2.imshow('potential obstacles', res_image)
        # cv2.waitKey(1)
        # # draw the new field_boundary line
        # for i in range(len(field_boundary_points) - 1):
        #     cv2.line(my_img, field_boundary_points[i], field_boundary_points[i+1], (0,255,255))
        # cv2.imshow('graham: input blue, output yellow', my_img)
        # cv2.waitKey(1)

        self._convex_field_boundary_points = field_boundary_points

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
            # cv2.line(my_img, stack[-1], stack[-2], (255,0,0),2)
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

    def _mask_field_boundary(self):
        """
        This is an old version that is not used anymore
        :return:
        """
        mask = self._color_detector.mask_image(self._image)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        field_boundary_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set field_boundary point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if np.mean(mask[max(0, y - 2):(y + 3),
                           max(0, x - 2):(x + 3)]) > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            field_boundary_points.append((x, firstgreen))
        return field_boundary_points

    def _precise_field_boundary(self):
        # type: () -> list
        """
        This is an old version that is not used anymore

        Calculates the field_boundary coordinates in a precise way, but less fast and efficient.
        It checks after having found a field_boundary if coordinates around this point are also green
        and thus under the field_boundary.
        It additionally employs checking between the last point known as not the field_boundary and the field_boundary point
        to see if the field_boundary starts somewhere in between. (Currently actually a TODO)
        see also: _fast_field_boundary()
        :return list of coordinates of the field_boundary:
        """
        # worst case:
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        field_boundary_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set field_boundary point to worst case
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
            field_boundary_points.append((x, firstgreen))
        return field_boundary_points

    def compute_full_field_boundary(self):
        if self._field_boundary_full is None:
            xp, fp = zip(*self.get_field_boundary_points())
            x = list(range(self._image.shape[1]))
            self._field_boundary_full = np.interp(x, list(xp), list(fp))

    def get_full_field_boundary(self):
        # type: () -> list
        """
        calculates an interpolated list of y coordinates where the field_boundary is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the field_boundary is. Index of y value is the x coordinate:
        """
        self.compute_full_field_boundary()
        return self._field_boundary_full

    def compute_full_convex_field_boundary(self):
        # type: () -> list
        """
        calculates an interpolated list of y coordinates where the convex field_boundary is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the convex field_boundary is. Index of y value is the x coordinate:
        """
        if self._convex_field_boundary_full is None:
            xp, fp = zip(*self.get_convex_field_boundary_points())
            x = list(range(self._image.shape[1]))
            self._convex_field_boundary_full = np.interp(x, list(xp), list(fp))

    def get_full_convex_field_boundary(self):
        # type: () -> list
        """
        calculates an interpolated list of y coordinates where the convex field_boundary is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the convex field_boundary is. Index of y value is the x coordinate:
        """
        self.compute_full_convex_field_boundary()
        return self._convex_field_boundary_full

    def candidate_under_field_boundary(self, candidate, y_offset=0):
        # type: (tuple, int) -> bool
        """
        returns whether the candidate is under the field_boundary or not
        :param candidate: the candidate, a tuple (upleft_x, upleft_y, width, height)
        :param y_offset: an offset in y-direction (higher offset allows points in a wider range over the field_boundary)
        :return: whether the candidate is under the field_boundary or not
        """
        footpoint = (candidate[0] + candidate[2] // 2, candidate[1] + candidate[3] + y_offset)
        return self.point_under_field_boundary(footpoint)

    def compute_all(self):
        self.compute_full_convex_field_boundary()
        self.compute_full_field_boundary()

    def candidates_under_field_boundary(self, candidates, y_offset=0):
        # type: (list, int) -> list
        return [candidate for candidate in candidates if self.candidate_under_field_boundary(candidate, y_offset)]

    def balls_under_field_boundary(self, balls, y_offset=0):
        # type: (list, int) -> list
        return [candidate for candidate in balls if self.candidate_under_field_boundary(
            (candidate.get_upper_left_x(),
             candidate.get_upper_left_y(),
             candidate.get_width(),
             candidate.get_height()),
            y_offset)]

    def point_under_field_boundary(self, point, offset=0):
        # type: (tuple, int) -> bool
        """
        returns if given coordinate is a point under field_boundary
        :param point: coordinate (x, y) to test
        :param offset: offset of pixels to still be accepted as under the field_boundary. Default is 0.
        :return a boolean if point is under field_boundary:
        """
        if not 0 <= point[0] < len(self.get_full_field_boundary()):
            rospy.logwarn('point_under_field_boundary got called with an out of bounds field_boundary point')
            return False
        return point[1] + offset > self.get_full_field_boundary()[point[0]]

    def get_upper_bound(self, y_offset=0):
        # type: () -> int
        """
        returns the y-value of highest point of the field_boundary (lowest y-value)
        :return: int(), y-value of highest point of the field_boundary (lowest y-value)
        """
        return max(0, int(min(self.get_field_boundary_points(), key=itemgetter(1))[1] - y_offset))

    def _equalize_points(self, points):
        # type: (list) -> list
        """
        returns a list of the input points with smoothed y-coordinates to reduce
        the impact of outlier points in the field_boundary, which are caused by
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
