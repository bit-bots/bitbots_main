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
from soccer_ipm.utils import create_horizontal_plane
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm


def map_ball_array(
        msg: sv2dm.BallArray,
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger,
        ball_diameter: float) -> sv3dm.BallArray:
    """
    Map a given array of 2D ball detections onto the ground plane.

    :param msg: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :param ball_diameter: The diameter of the balls that are mapped
    :returns: The balls as 3D cartesian detections in the output_frame
    """
    elevated_field = create_horizontal_plane(ball_diameter / 2)

    balls_relative = sv3dm.BallArray()
    balls_relative.header.stamp = msg.header.stamp
    balls_relative.header.frame_id = output_frame

    ball: sv2dm.Ball
    for ball in msg.balls:
        try:
            transformed_ball = ipm.map_point(
                elevated_field,
                ball.center,
                msg.header.stamp,
                plane_frame_id=output_frame,
                output_frame_id=output_frame)

            ball_relative = sv3dm.Ball()
            ball_relative.center = transformed_ball.point
            ball_relative.confidence = ball.confidence
            balls_relative.balls.append(ball_relative)
        except NoIntersectionError:
            logger.warn(
                f'Could not transform ball at ({ball.center.x},{ball.center.y}).',
                throttle_duration_sec=5)
    return balls_relative
