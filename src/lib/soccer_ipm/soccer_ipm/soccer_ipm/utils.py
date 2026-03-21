# Copyright (c) 2022 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections.abc import Callable
from functools import wraps

from ipm_library.exceptions import CameraInfoNotSetException
from rclpy.impl.rcutils_logger import RcutilsLogger
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import Plane
import tf2_ros as tf2
from vision_msgs.msg import BoundingBox2D, Point2D


def create_horizontal_plane(
        height_offset: float = 0.0) -> Plane:
    """Create a plane message for a given frame at a given time, with a given height offset."""
    plane = Plane()
    plane.coef[2] = 1.0  # Normal in z direction
    plane.coef[3] = -height_offset  # Distance above the ground
    return plane


def object_at_bottom_of_image(
            camera_info: CameraInfo,
            position: float,
            thresh: float) -> bool:
    """
    Check if the objects y position is at the bottom of the image.

    :param camera_info: The current CameraInfo message
    :param position: Y-position of the object
    :param thresh: Threshold defining the region at the bottom of the image which is
        counted as 'the bottom' as a fraction of the image height
    """
    image_height = camera_info.height
    image_height /= max(camera_info.binning_y, 1)

    scaled_thresh = thresh * image_height
    return position > scaled_thresh


def bb_footpoint(bounding_box: BoundingBox2D) -> Point2D:
    """
    Return the footpoint of a given bounding box.

    :param bounding_box: The bounding box
    :returns: The footpoint (aka. the bottom center)
    """
    return Point2D(
        x=float(bounding_box.center.position.x),
        y=float(bounding_box.center.position.y + bounding_box.size_y // 2),
    )


def catch_camera_info_not_set_error(logger: RcutilsLogger) -> Callable:
    """
    Camera info exception handling decorator factory.

    See also https://www.artima.com/weblogs/viewpost.jsp?thread=240845

    :param logger: The rclpy logger that prints the warning
    :returns: The decorator that utilizes the logger
    """
    def wrapped_decorator(f: Callable) -> Callable:
        @wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except CameraInfoNotSetException:
                logger.warn(
                    'Inverse perspective mapping should be performed, '
                    'but no camera info was recived yet!',
                    throttle_duration_sec=5)
        return wrapper
    return wrapped_decorator


def catch_tf_timing_error(logger: RcutilsLogger) -> Callable:
    """
    TF exception handling decorator factory.

    See also https://www.artima.com/weblogs/viewpost.jsp?thread=240845

    :param logger: The rclpy logger that prints the warning
    :returns: The decorator that utilizes the logger
    """
    def wrapped_decorator(f: Callable) -> Callable:
        @wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except (tf2.ConnectivityException,
                    tf2.LookupException,
                    tf2.ExtrapolationException) as e:
                logger.warn(
                    'Inverse perspective mapping should be performed, '
                    f'but a tf error occured: {str(e)}',
                    throttle_duration_sec=5)
        return wrapper
    return wrapped_decorator
