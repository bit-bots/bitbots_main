import numpy as np
import cv2
import rospy
import abc
import math
import tf2_ros as tf2
from tf.transformations import euler_from_quaternion
from .color import ColorDetector
from operator import itemgetter


class FieldBoundaryDetector(object):
    def __init__(self, config, field_color_detector):
        # type: (dict, ColorDetector) -> None
        """
        This is the abctract class for the field boundary detector.

        :param config: the configuration contained in visionparams.yaml
        :param field_color_detector: checks whether a color is part of the field colors
        """
        # set variables:
        self._image = None
        self._field_boundary_points = None
        self._field_boundary_full = None
        self._convex_field_boundary_points = None
        self._convex_field_boundary_full = None
        self._mask = None
        self._field_color_detector = field_color_detector
        # init config:
        self._x_steps = config['field_boundary_detector_horizontal_steps']
        self._y_steps = config['field_boundary_detector_vertical_steps']
        self._roi_height = config['field_boundary_detector_roi_height']
        self._roi_width = config['field_boundary_detector_roi_width']
        self._roi_increase = config['field_boundary_detector_roi_increase']
        self._green_threshold = config['field_boundary_detector_green_threshold']

    @staticmethod
    def get_by_name(search_method):
        # type: (String) -> FieldBoundaryDetector
        """
        Returns the matching field boundary detector for an String.

        :param image: the current frame of the video feed
        """
        detectors = {
            'dynamic': DynamicFieldBoundaryDetector,
            'binary': BinaryFieldBoundaryDetector,
            'reversed': ReversedFieldBoundaryDetector,
            'cvreversed': CVReversedFieldBoundaryDetector,
            'iteration': IterationFieldBoundaryDetector,
        }
        return detectors[search_method]

    def set_image(self, image):
        # type: (np.matrix) -> None
        """
        Refreshes the variables after receiving an image.

        :param image: the current frame of the video feed
        """
        self._image = image
        self._field_boundary_points = None
        self._field_boundary_full = None
        self._convex_field_boundary_full = None
        self._convex_field_boundary_points = None
        self._mask = None

    def get_mask(self):
        # type: () -> np.array
        """
        :return: np.array
        """
        # Compute mask (cached)
        self._compute_mask()
        return self._mask

    def _compute_mask(self):
        # type: () -> None
        """
        Calculates a mask that contains white pixels below the field-boundary
        """
        # Check if field boundary is already cached
        if self._mask is None:
            shape = np.shape(self._image)
            img_size = (shape[0], shape[1])
            # Generates a white canvas
            canvas = np.ones(img_size, dtype=np.uint8) * 255
            hpoints = np.array([[(0, 0)] + self.get_field_boundary_points() + [(shape[1] - 1, 0)]])
            # Blacks out the part over the field_boundary
            self._mask = cv2.fillPoly(canvas, hpoints, 0)

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

    @abc.abstractmethod
    def _compute_field_boundary_points(self):
        """
        calls the method to compute the field boundary points and saves it in the class variable _field_boundary_points
        """
        raise NotImplementedError

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

        # calculate the "convex hull" of the field_boundary points
        self._convex_field_boundary_points = self._graham(field_boundary_points)

    def _graham(self, points):
        '''
        This is a modified Graham's convex hull algorithm. Instead of returning the list
        of points that form the entire convex hull of the input point set, it returns
        only the "half" of the hull which has the lower y-coordinates and spans between the
        points with x=0 and x=self._image.shape[1]-1.

        :param points:  list of points (a point is a 2D array (x,y)) with increasing x-coordinates,
                        including one point with x = 0 and one point with x = self._image.shape[1]-1
        :return: list of points, see above for more detail
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

        i = 2
        while (i < num_points) and (stack[-1][0] != self._image.shape[1] - 1):

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

    def _compute_full_field_boundary(self):
        if self._field_boundary_full is None:
            xp, fp = zip(*self.get_field_boundary_points())
            x = list(range(self._image.shape[1]))
            self._field_boundary_full = np.interp(x, list(xp), list(fp))

    def get_full_field_boundary(self):
        # type: () -> list
        """
        Calculates an interpolated list of y coordinates where the field_boundary is for the picture
        the index of the y value is the x coordinate on the picture.

        :return list of y coordinates where the field_boundary is. Index of y value is the x coordinate:
        """
        self._compute_full_field_boundary()
        return self._field_boundary_full

    def _compute_full_convex_field_boundary(self):
        # type: () -> list
        """
        Calculates an interpolated list of y coordinates where the convex field_boundary is for the picture
        the index of the y value is the x coordinate on the picture.

        :return list of y coordinates where the convex field_boundary is. Index of y value is the x coordinate:
        """
        if self._convex_field_boundary_full is None:
            xp, fp = zip(*self.get_convex_field_boundary_points())
            x = list(range(self._image.shape[1]))
            self._convex_field_boundary_full = np.interp(x, list(xp), list(fp))

    def get_full_convex_field_boundary(self):
        # type: () -> list
        """
        Calculates an interpolated list of y coordinates where the convex field_boundary is for the picture
        the index of the y value is the x coordinate on the picture.

        :return list of y coordinates where the convex field_boundary is. Index of y value is the x coordinate:
        """
        self._compute_full_convex_field_boundary()
        return self._convex_field_boundary_full

    def candidate_under_field_boundary(self, candidate, y_offset=0):
        # type: (tuple, int) -> bool
        """
        Returns whether the candidate is under the field_boundary or not.

        :param candidate: the candidate
        :param y_offset: an offset in y-direction (higher offset allows points in a wider range over the field_boundary)
        :return: whether the candidate is under the field_boundary or not
        """
        footpoint = candidate.get_lower_center_point()
        footpoint_with_offset = (footpoint[0], footpoint[1] + y_offset)
        return self.point_under_field_boundary(footpoint_with_offset)

    def candidate_under_convex_field_boundary(self, candidate, y_offset=0):
        # type: (tuple, int) -> bool
        """
        Returns whether the candidate is under the convex field_boundary or not.

        :param candidate: the candidate
        :param y_offset: an offset in y-direction (higher offset allows points in a wider range over the field_boundary)
        :return: whether the candidate is under the convex field_boundary or not
        """
        footpoint = candidate.get_lower_center_point()
        footpoint_with_offset = (footpoint[0], footpoint[1] + y_offset)
        return self.point_under_convex_field_boundary(footpoint_with_offset)

    def candidates_under_field_boundary(self, candidates, y_offset=0):
        # type: (list, int) -> list
        """
        Removes candidates that are not under the field boundary from list.

        :param balls: list of all candidates
        :param y_offset: If the ball is within this offset over the field boundary its still accepted.
        :return: list of candidates under the field boundary
        """
        return [candidate for candidate in candidates if self.candidate_under_field_boundary(candidate, y_offset)]

    def candidates_under_convex_field_boundary(self, candidates, y_offset=0):
        # type: (list, int) -> list
        """
        Removes candidates that are not under the convex field boundary from list.

        :param balls: list of all candidates
        :param y_offset: If the ball is within this offset over the field boundary its still accepted.
        :return: list of candidates under convex the field boundary
        """
        return [candidate for candidate in candidates if self.candidate_under_convex_field_boundary(candidate, y_offset)]

    def point_under_field_boundary(self, point, offset=0):
        # type: (tuple, int) -> bool
        """
        Returns if given coordinate is a point under field_boundary.

        :param point: coordinate (x, y) to test
        :param offset: offset of pixels to still be accepted as under the field_boundary. Default is 0.
        :return a boolean if point is under field_boundary:
        """
        if not 0 <= point[0] < len(self.get_full_field_boundary()):
            rospy.logwarn('point_under_field_boundary got called with an out of bounds field_boundary point', logger_name="vision_field_boundary")
            return False
        return point[1] + offset > self.get_full_field_boundary()[point[0]]

    def point_under_convex_field_boundary(self, point, offset=0):
        # type: (tuple, int) -> bool
        """
        Returns if given coordinate is a point under the convex field_boundary.

        :param point: coordinate (x, y) to test
        :param offset: offset of pixels to still be accepted as under the field_boundary. Default is 0.
        :return a boolean if point is under the convex field_boundary:
        """
        if not 0 <= point[0] < len(self.get_full_convex_field_boundary()):
            rospy.logwarn('point_under_field_boundary got called with an out of bounds field_boundary point', logger_name="vision_field_boundary")
            return False
        return point[1] + offset > self.get_full_convex_field_boundary()[point[0]]

    def get_upper_bound(self, y_offset=0):
        # type: () -> int
        """
        Returns the y-value of highest point of the field_boundary (lowest y-value).

        :return: int(), y-value of highest point of the field_boundary (lowest y-value)
        """
        return max(0, int(min(self.get_field_boundary_points(), key=itemgetter(1))[1] - y_offset))

    def _equalize_points(self, points):
        # type: (list) -> list
        """
        Returns a list of the input points with smoothed y-coordinates to reduce
        the impact of outlier points in the field_boundary, which are caused by
        detection errors.

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


class IterationFieldBoundaryDetector(FieldBoundaryDetector):
    def __init__(self, config, field_color_detector):
        """
        This is the iteration field boundary detector.
        It uses the iteration detection method and finds the field boundary via scan lines running down from top to bottom.

        :param config: the configuration contained in visionparams.yaml
        :param field_color_detector: checks whether a color is part of the field colors
        """
        super(IterationFieldBoundaryDetector, self).__init__(config, field_color_detector)

    def _compute_field_boundary_points(self):
        """
        Calls the method to compute the field boundary via the iteration method and saves it in the class variable _field_boundary_points
        """
        # Calc field boundary
        self._field_boundary_points = IterationFieldBoundaryAlgorithm._calculate_field_boundary(
            self._image,
            self._field_color_detector,
            self._x_steps,
            self._y_steps,
            self._roi_height,
            self._roi_width,
            self._roi_increase,
            self._green_threshold)


class BinaryFieldBoundaryDetector(FieldBoundaryDetector):
    def __init__(self, config, field_color_detector):
        """
        This is the binary search field boundary detector.
        It uses the binary detection method and finds the field boundary via binary search.

        :param config: the configuration contained in visionparams.yaml
        :param field_color_detector: checks whether a color is part of the field colors
        """
        super(BinaryFieldBoundaryDetector, self).__init__(config, field_color_detector)

    def _compute_field_boundary_points(self):
        """
        Calls the method to compute the field boundary via the binary search and saves it in the class variable _field_boundary_points
        """
        # Calc field boundary
        self._field_boundary_points = BinaryFieldBoundaryAlgorithm._calculate_field_boundary(
            self._image,
            self._field_color_detector,
            self._x_steps,
            self._y_steps,
            self._roi_height,
            self._roi_width,
            self._roi_increase,
            self._green_threshold)


class ReversedFieldBoundaryDetector(FieldBoundaryDetector):
    def __init__(self, config, field_color_detector):
        """
        This is the reversed iteration field boundary detector.
        It uses the reversed detection method and finds the field boundary via scan lines running up from bottom to top.

        :param config: the configuration contained in visionparams.yaml
        :param field_color_detector: checks whether a color is part of the field colors
        """
        super(ReversedFieldBoundaryDetector, self).__init__(config, field_color_detector)

    def _compute_field_boundary_points(self):
        """
        Calls the method to compute the field boundary via the reversed iteration method and saves it in the class variable _field_boundary_points
        """
        # Calc field boundary
        self._field_boundary_points = ReversedFieldBoundaryAlgorithm._calculate_field_boundary(
            self._image,
            self._field_color_detector,
            self._x_steps,
            self._y_steps,
            self._roi_height,
            self._roi_width,
            self._roi_increase,
            self._green_threshold)


class CVReversedFieldBoundaryDetector(FieldBoundaryDetector):
    def __init__(self, config, field_color_detector):
        """
        This is the reversed iteration field boundary detector implemented in OpenCV.
        It uses the reversed detection method and finds the field boundary via scan lines running up from bottom to top.

        :param config: the configuration contained in visionparams.yaml
        :param field_color_detector: checks whether a color is part of the field colors
        """
        super(CVReversedFieldBoundaryDetector, self).__init__(config, field_color_detector)

    def _compute_field_boundary_points(self):
        """
        Calls the method to compute the field boundary via the reversed iteration method and saves it in the class variable _field_boundary_points
        """
        # Calc field boundary
        self._field_boundary_points = CVReversedFieldBoundaryAlgorithm._calculate_field_boundary(
            self._image,
            self._field_color_detector,
            self._x_steps,
            self._y_steps,
            self._roi_height,
            self._roi_width,
            self._roi_increase,
            self._green_threshold)


class DynamicFieldBoundaryDetector(FieldBoundaryDetector):
    def __init__(self, config, field_color_detector):
        """
        This is the dynamic field boundary detector.
        It switches between the iteration and reversed iteration method. It depends on how much the robot head is tilted.
        This improves performance (iteration) and enables operation with two field next to each other (reversed).

        :param config: the configuration contained in visionparams.yaml
        :param field_color_detector: checks whether a color is part of the field colors
        """
        super(DynamicFieldBoundaryDetector, self).__init__(config, field_color_detector)

        self._over_horizon_algorithm = ReversedFieldBoundaryAlgorithm
        self._under_horizon_algorithm = IterationFieldBoundaryAlgorithm
        self._base_frame = "camera_optical_frame"
        self._camera_frame = "base_footprint"
        self._tilt_threshold = math.radians(config['field_boundary_detector_head_tilt_threshold'])

        # TF stuff
        self._tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5))
        self._tf_listener = tf2.TransformListener(self._tf_buffer)

    def _only_field_visible(self):
        """
        Check head orientation and decide if we should use the iteration or reversed iteration method.
        """
        # Check if we can use tf. Otherwise switch to reversed iteration detector
        try:
            # Get quaternion from newest tf
            orientation = self._tf_buffer.lookup_transform(self._camera_frame, self._base_frame, rospy.Time(0)).transform.rotation
            # Convert into an usable tilt angle
            tilt_angle =  (1.5 * math.pi - euler_from_quaternion((
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w))[0]) % (2 * math.pi)
            # Check if it satisfied the threshold
            if tilt_angle > self._tilt_threshold and tilt_angle < math.pi:
                return True
            else:
                return False
        # Switch to reversed iteration detector
        except tf2.LookupException:
            rospy.logwarn_throttle(2, "TF for dynamic field boundary algorithm selection not active. Maybe TF becomes avalabile in a few seconds. Using reversed iteration method instead",
                logger_name="vision_field_boundary")
            return False
        except tf2.ExtrapolationException as ecp:
            # Warn user
            rospy.logwarn_throttle(2, "Extrapolation exception! Not able to use tf for dynamic field boundary algorithm selection. Using reversed iteration method instead",
                logger_name="vision_field_boundary")
            return False
        except tf2.ConnectivityException as ecp:
            # Warn user
            rospy.logwarn_throttle(2, "Connectivity exception! Not able to use tf for dynamic field boundary algorithm selection. Using reversed iteration method instead. \n" + ecp)
            return False

    def _compute_field_boundary_points(self):
        """
        Calls the method to compute the field boundary and saves it in the class variable _field_boundary_points
        """
        if self._only_field_visible():
            selected_algorithm = self._under_horizon_algorithm
        else:
            selected_algorithm = self._over_horizon_algorithm
        # Calc field boundary
        self._field_boundary_points = selected_algorithm._calculate_field_boundary(
            self._image,
            self._field_color_detector,
            self._x_steps,
            self._y_steps,
            self._roi_height,
            self._roi_width,
            self._roi_increase,
            self._green_threshold)

class FieldBoundaryAlgorithm():
    """
    Definition of the interface for an field boundary algorithm
    """
    @abc.abstractmethod
    def _calculate_field_boundary(_image, _field_color_detector, _x_steps, _y_steps, _roi_height, _roi_width, _roi_increase, _green_threshold):
        """
        Calculates (if implemented) the field boundary
        :returns: list of field boundary points
        """
        raise NotImplementedError


class IterationFieldBoundaryAlgorithm(FieldBoundaryAlgorithm):
    @staticmethod
    def _calculate_field_boundary(_image, _field_color_detector, _x_steps, _y_steps, _roi_height, _roi_width, _roi_increase, _green_threshold):
        # type: () -> list
        """
        Finds the points of the field boundary visible in the image. Uses the standard method iterating from top to
        bottom until it finds enough green points.

        :returns: list of field boundary points
        """

        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = _field_color_detector.get_mask_image()
        # noise reduction on the field_mask:
        field_mask = cv2.morphologyEx(
            field_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)

        # Syntax: cv2.resize(image, (width, height), type of interpolation)
        field_mask = cv2.resize(field_mask, (_x_steps, _y_steps), interpolation=cv2.INTER_LINEAR)

        # the stepsize is the number of pixels traversed in the image by going one step
        y_stepsize = (_image.shape[0] - 1) / float(_y_steps - 1)
        x_stepsize = (_image.shape[1] - 1) / float(_x_steps - 1)

        min_y = _image.shape[0] - 1

        _field_boundary_points = []
        for x_step in range(_x_steps):  # traverse columns
            firstgreen = min_y  # set field_boundary point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(_y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if field_mask[y_step, x_step] > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            _field_boundary_points.append((x, firstgreen))
        return _field_boundary_points


class ReversedFieldBoundaryAlgorithm(FieldBoundaryAlgorithm):
    @staticmethod
    def _calculate_field_boundary(_image, _field_color_detector, _x_steps, _y_steps, _roi_height, _roi_width, _roi_increase, _green_threshold):
        # type: () -> list
        """
        Finds the points of the field boundary visible in the image. Uses the reversed method iterating from bottom to
        top until it finds enough non green points. Useful for when two fields are adjacent to each other.

        :returns: list of field boundary points
        """

        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = _field_color_detector.get_mask_image()
        # noise reduction on the field_mask:
        field_mask = cv2.morphologyEx(
            field_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)

        # the stepsize is the number of pixels traversed in the image by going one step
        y_stepsize = (_image.shape[0] - 1) / float(_y_steps - 1)
        x_stepsize = (_image.shape[1] - 1) / float(_x_steps - 1)

        # the region of interest (roi) for a specific point is a rectangle with the point in the middle of its top row
        # the point is slightly left of center when the width is even
        roi_start_height_y = _roi_height
        roi_start_width_x = _roi_width
        roi_start_radius_x = roi_start_width_x // 2
        # increase of roi radius per pixel, e.g. 0.1 increases the radius by 1 for every 10 pixels
        # this accommodates for size difference of objects in the image depending on their distance
        # and therefore height in image:
        roi_increase = _roi_increase
        # height/width/radius-in-x-direction of the roi after maximum increase at the bottom of the image
        roi_max_height_y = roi_start_height_y + int(_image.shape[0] * roi_increase * 2)
        roi_max_width_x = roi_start_width_x + int(_image.shape[0] * roi_increase * 2)
        roi_max_radius_x = roi_max_width_x // 2
        # extents the outermost pixels of the image as the roi will go beyond the image at the edges
        # Syntax: cv2.copyMakeBorder(image, top, bottom, left, right, type of extension)
        field_mask = cv2.copyMakeBorder(field_mask, roi_start_height_y, 0, roi_max_radius_x, roi_max_radius_x,
                                        cv2.BORDER_REPLICATE)

        # uncomment this to use a kernel for the roi
        kernel = np.ones((roi_max_height_y, roi_max_width_x))  # creates a kernel with 0 everywhere
        # use this to fill in other values at specific places in the kernel
        kernel[0: int(roi_max_height_y // 5), int(roi_max_width_x // 2.2): int(roi_max_width_x - roi_max_width_x // 2.2)] = 10

        green_threshold = _green_threshold

        _field_boundary_points = []
        for x_step in range(_x_steps):  # traverse columns (x_steps)
            top_green = roi_start_height_y  # set field_boundary point to worst case
            # calculate the x coordinate in the image of a column:
            x_image = int(round(x_step * x_stepsize)) + roi_max_radius_x
            for y_step in range(_y_steps):
                # calculate the y coordinate in the image of a y_step:
                y_image = int(_image.shape[0] - (round(y_step * y_stepsize))) + roi_start_height_y

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
            _field_boundary_points.append((x_image - roi_max_radius_x, top_green))
        return _field_boundary_points


class CVReversedFieldBoundaryAlgorithm(FieldBoundaryAlgorithm):
    @staticmethod
    def _calculate_field_boundary(image, field_color_detector, x_steps, y_steps, roi_height, roi_width, roi_increase, green_threshold):
        # type: () -> list
        """
        Finds the points of the field boundary visible in the image. Uses the reversed method iterating from bottom to
        top until it finds enough non green points. Useful for when two fields are adjacent to each other.

        :returns: list of field boundary points
        """
        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = field_color_detector.get_mask_image()

        # Scale the image down
        subsampled_mask = cv2.resize(field_mask,(x_steps,y_steps), interpolation=cv2.INTER_AREA)
        # Blur the downscaled image to fill holes in the field mask
        subsampled_mask = cv2.GaussianBlur(subsampled_mask,(3,3),0)

        field_boundary_points = []

        # Iterate horizontal over the image
        for x_position in range(subsampled_mask.shape[1]):
            # Iterate vertical over the downscaled mask
            for y_position in range(subsampled_mask.shape[0]):
                # Invert the current vertical value
                max_y = (subsampled_mask.shape[0] - 1) - y_position
                # Check if we found a pixel under the threshold
                if subsampled_mask[max_y, x_position] < int(green_threshold/1000 * 255):
                    # Reset to last step
                    max_y += 1
                    break

            # Scale the stuff for the original image
            field_boundary_points.append(
                (int((x_position+0.5)*(field_mask.shape[1]/x_steps)),
                (int(max_y*(field_mask.shape[0]/y_steps)))))
        # Fix tuff at the image edges
        field_boundary_points[0] = (0, field_boundary_points[0][1])
        field_boundary_points[-1] = (field_mask.shape[1]-1, field_boundary_points[0][1])
        return field_boundary_points



class BinaryFieldBoundaryAlgorithm(FieldBoundaryAlgorithm):
    @staticmethod
    def _calculate_field_boundary(_image, _field_color_detector, _x_steps, _y_steps, _roi_height, _roi_width, _roi_increase, _green_threshold):
        """
        Finds the points of the field edge visible in the image. Uses a faster binary search method, that unfortunately
        finds these points below field lines sometimes.

        :returns: list of field boundary points
        """
        # calculate the field_mask which contains 0 for non-green pixels and 255 for green pixels in the image
        # index counting up from top to bottom and left to right
        field_mask = _field_color_detector.get_mask_image()
        # noise reduction on the field_mask:
        field_mask = cv2.morphologyEx(
            field_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)

        # the stepsize is the number of pixels traversed in the image by going one step
        y_stepsize = (_image.shape[0] - 1) / float(_y_steps - 1)
        x_stepsize = (_image.shape[1] - 1) / float(_x_steps - 1)

        # the region of interest (roi) for a specific point is a rectangle
        # with the point in the middle of its top row; the point is slightly left of center when the width is even
        roi_start_height_y = _roi_height
        roi_start_width_x = _roi_width
        roi_start_radius_x = roi_start_width_x // 2
        # increase of roi radius per pixel, e.g. 0.1 increases the radius by 1 for every 10 pixels
        # this accommodates for size difference of objects in the image depending on their distance
        # and therefore height in image:
        roi_increase = _roi_increase
        # height/width/radius-in-x-direction of the roi after maximum increase at the bottom of the image
        roi_max_height_y = roi_start_height_y + int(_image.shape[0] * roi_increase * 2)
        roi_max_width_x = roi_start_width_x + int(_image.shape[0] * roi_increase * 2)
        roi_max_radius_x = roi_max_width_x // 2
        # extents the outermost pixels of the image as the roi will go beyond the image at the edges
        # Syntax: cv2.copyMakeBorder(image, top, bottom, left, right, type of extension)
        field_mask = cv2.copyMakeBorder(field_mask, 0, roi_max_height_y, roi_max_radius_x, roi_max_radius_x,
                                        cv2.BORDER_REPLICATE)

        # uncomment this to use a kernel for the roi
        # kernel = np.zeros((roi_height, roi_width))  # creates a kernel with 0 everywhere
        # kernel[int(kernel_height/2):, :] = 1  # use this to fill in other values at specific places in the kernel

        green_threshold = _green_threshold

        _field_boundary_points = []

        for x_step in range(0, _x_steps):  # traverse columns (x_steps)
            roi_mean = 0
            y_step = 0
            roi_current_height_y = 0
            # binary search for finding the first green pixel:
            first = 0  # top of the column
            last = _y_steps - 1  # bottom of the column
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
            _field_boundary_points.append((x_image - roi_max_radius_x, y_image))
        return _field_boundary_points
