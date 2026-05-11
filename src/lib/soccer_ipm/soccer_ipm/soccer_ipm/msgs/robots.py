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

from ipm_library.exceptions import NoIntersectionError
from ipm_library.ipm import IPM
from rclpy.impl.rcutils_logger import RcutilsLogger
from soccer_ipm.utils import (bb_footpoint, create_horizontal_plane, object_at_bottom_of_image)
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm


def map_robot_array(
        msg: sv2dm.RobotArray,
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger,
        footpoint_out_of_image_threshold: float,
        object_default_dimensions: tuple[float, float, float]) -> sv3dm.RobotArray:
    """
    Map a given array of 2D robot detections onto the field plane.

    :param msg: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :param footpoint_out_of_image_threshold: Size of the area at the bottom of the image at which
        the object is considered to be only partially visible
    :param object_default_dimensions: Default dimensions of the 3D objects
    :returns: The robots as 3D cartesian detections in the output_frame
    """
    field = create_horizontal_plane()

    robots = sv3dm.RobotArray()
    robots.header.stamp = msg.header.stamp
    robots.header.frame_id = output_frame

    robot: sv2dm.Robot
    for robot in msg.robots:

        # Check if post is not going out of the image at the bottom
        if not object_at_bottom_of_image(
                ipm.get_camera_info(),
                bb_footpoint(robot.bb).y,
                footpoint_out_of_image_threshold):
            # Create footpoint
            footpoint = bb_footpoint(robot.bb)
            # Map point from image onto field plane
            try:
                relative_foot_point = ipm.map_point(
                    field,
                    footpoint,
                    msg.header.stamp,
                    plane_frame_id=output_frame,
                    output_frame_id=output_frame)

                transformed_robot = sv3dm.Robot()
                transformed_robot.attributes = robot.attributes
                transformed_robot.confidence = robot.confidence
                transformed_robot.bb.size.x = object_default_dimensions[0]
                transformed_robot.bb.size.y = object_default_dimensions[1]
                transformed_robot.bb.size.z = object_default_dimensions[2]
                transformed_robot.bb.center.position = relative_foot_point.point
                transformed_robot.bb.center.position.z += transformed_robot.bb.size.z / 2
                robots.robots.append(transformed_robot)
            except NoIntersectionError:
                logger.warn(
                    'Got a robot with foot point ({},{}) I could not transform.'.format(
                        footpoint.x,
                        footpoint.y),
                    throttle_duration_sec=5)
    return robots
