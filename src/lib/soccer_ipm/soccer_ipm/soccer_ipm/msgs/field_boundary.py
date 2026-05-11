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

from geometry_msgs.msg import Point
from ipm_library.ipm import IPM
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from soccer_ipm.utils import create_horizontal_plane
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm


def map_field_boundary(
        msg: sv2dm.FieldBoundary,
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger) -> sv3dm.FieldBoundary:
    """
    Map a 2D field boundary in the image onto the field plane.

    :param msg: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :returns: The 3D cartesian points of the field boundary in the output_frame
    """
    field = create_horizontal_plane()

    field_boundary = sv3dm.FieldBoundary()
    field_boundary.header.stamp = msg.header.stamp
    field_boundary.header.frame_id = output_frame
    field_boundary.confidence = msg.confidence

    # Convert points to numpy array
    points_np = np.array([[p.x, p.y] for p in msg.points])

    # Map all points at once from image onto field plane
    points_on_plane = ipm.map_points(
        field,
        points_np,
        msg.header.stamp,
        plane_frame_id=output_frame,
        output_frame_id=output_frame)[1]

    # Check for invalid field boundary points and convert back from numpy
    for p in points_on_plane:
        # Check if any part of the point is nan
        if np.any(np.isnan(p)):
            logger.warn(
                'Got a field boundary point I could not transform.',
                throttle_duration_sec=5)
            continue
        # Convert back from numpy
        field_boundary.points.append(Point(x=p[0], y=p[1], z=p[2]))
    return field_boundary
