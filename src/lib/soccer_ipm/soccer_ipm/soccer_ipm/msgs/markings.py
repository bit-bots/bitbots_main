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

from geometry_msgs.msg import Vector3
from ipm_library.exceptions import NoIntersectionError
from ipm_library.ipm import IPM
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from soccer_ipm.utils import create_horizontal_plane
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm
from std_msgs.msg import Header
from vision_msgs.msg import Point2D


def map_marking_array(
        msg: sv2dm.MarkingArray,
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger) -> sv3dm.MarkingArray:
    """
    Map a given array of 2D field marking detections onto the field plane.

    :param msg: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :returns: The markings as 3D cartesian detections in the output_frame
    """
    markings = sv3dm.MarkingArray()
    markings.header.stamp = msg.header.stamp
    markings.header.frame_id = output_frame
    markings.intersections = map_marking_intersections(
        msg.header,
        msg.intersections,
        ipm,
        output_frame,
        logger)
    markings.segments = map_marking_segments(
        msg.header,
        msg.segments,
        ipm,
        output_frame,
        logger)
    markings.ellipses = map_marking_ellipses(
        msg.header,
        msg.ellipses,
        ipm,
        output_frame,
        logger)
    return markings


def map_marking_intersections(
        header: Header,
        intersections_2d: list[sv2dm.MarkingIntersection],
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger) -> list[sv3dm.MarkingIntersection]:
    """
    Map a given list of 2D field marking intersections onto the field plane.

    :param header: Message header of the original detection
    :param intersections_2d: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :returns: The intersections as 3D cartesian detections in the output_frame
    """
    field = create_horizontal_plane()

    intersections_3d = []

    intersection: sv2dm.MarkingIntersection
    for intersection in intersections_2d:
        # Map point from image onto field plane
        try:
            mapped_center_point = ipm.map_point(
                field,
                intersection.center,
                header.stamp,
                plane_frame_id=output_frame,
                output_frame_id=output_frame)
            mapped_intersection = sv3dm.MarkingIntersection()
            mapped_intersection.center = mapped_center_point.point
            mapped_intersection.confidence = intersection.confidence
            mapped_intersection.num_rays = intersection.num_rays

            # Project rays
            for ray in intersection.heading_rays:
                # Create center point offset by the heading vector in image space
                ray_end_point = Point2D(
                    x=intersection.center.x + np.sin(ray),
                    y=intersection.center.y + np.cos(ray)
                )
                # Map newly optained end point of the ray
                mapped_ray_end = ipm.map_point(
                    field,
                    ray_end_point,
                    header.stamp,
                    plane_frame_id=output_frame,
                    output_frame_id=output_frame)
                # Substract the center point from the ray end point to get the vector
                ray_mapped = Vector3(
                    x=mapped_ray_end.point.x - mapped_center_point.point.x,
                    y=mapped_ray_end.point.y - mapped_center_point.point.y,
                    z=mapped_ray_end.point.z - mapped_center_point.point.z
                )
                mapped_intersection.rays.append(ray_mapped)
            intersections_3d.append(mapped_intersection)
        except NoIntersectionError:
            logger.warn(
                'Got a obstacle with foot point ({},{}) I could not transform.'.format(
                    mapped_center_point.point.x,
                    mapped_center_point.point.y),
                throttle_duration_sec=5)
    return intersections_3d


def map_marking_segments(
        header: Header,
        marking_segments_2d: list[sv2dm.MarkingSegment],
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger) -> list[sv3dm.MarkingSegment]:
    """
    Map a given list of 2D field marking segments onto the field plane.

    :param header: Message header of the original detection
    :param intersections_2d: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :returns: The segments as 3D cartesian detections in the output_frame
    """
    field = create_horizontal_plane()

    # Convert segment start and end points to np array
    segment_points_np = np.array([(
        (segment.start.x, segment.start.y),
        (segment.end.x, segment.end.y)) for segment in marking_segments_2d])

    # Map all points at once from image onto field plane
    segments_on_plane = ipm.map_points(
        field,
        segment_points_np.reshape(-1, 2),
        header.stamp,
        plane_frame_id=output_frame,
        output_frame_id=output_frame)[1].reshape(-1, 2, 3)

    # Convert the numpy array back to the soccer vision messages datatype
    marking_segments_3d = []
    for i, segment in enumerate(segments_on_plane):
        # Check if any of the points failed to map
        if np.any(np.isnan(segment)):
            logger.warn(
                'Got a segment I could not transform.',
                throttle_duration_sec=5)
            continue
        start, end = segment
        segment_msg = sv3dm.MarkingSegment()
        # Get confidence value from original list
        segment_msg.confidence.confidence = marking_segments_2d[i].confidence.confidence
        # Convert np array -> Point
        segment_msg.start.x = start[0]
        segment_msg.start.y = start[1]
        segment_msg.start.z = start[2]
        segment_msg.end.x = end[0]
        segment_msg.end.y = end[1]
        segment_msg.end.z = end[2]
        marking_segments_3d.append(segment_msg)
    return marking_segments_3d


def map_marking_ellipses(
        header: Header,
        ellipses_2d: list[sv2dm.MarkingEllipse],
        ipm: IPM,
        output_frame: str,
        logger: RcutilsLogger) -> list[sv3dm.MarkingEllipse]:
    """
    Map a given list of 2D field marking ellipses onto the field plane.

    :param header: Message header of the original detection
    :param intersections_2d: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param output_frame: The tf frame of the field
    :param logger: A ros logger to display warnings etc.
    :returns: The ellipses as 3D cartesian detections in the output_frame
    """
    field = create_horizontal_plane()

    ellipses_3d = []
    ellipse: sv2dm.MarkingEllipse
    for ellipse in ellipses_2d:
        diff = ellipse.bb.center.position.x - ellipse.center.x
        radius = ellipse.bb.size_x // 2 + diff
        side_point = Point2D(
            x=ellipse.center.x + radius,
            y=ellipse.center.y
        )

        # Map point from image onto field plane
        try:
            mapped_center_point = ipm.map_point(
                field,
                ellipse.center,
                header.stamp,
                plane_frame_id=output_frame,
                output_frame_id=output_frame)
            mapped_side_point = ipm.map_point(
                field,
                side_point,
                header.stamp,
                plane_frame_id=output_frame,
                output_frame_id=output_frame)
            mapped_ellipse = sv3dm.MarkingEllipse()
            mapped_ellipse.center.position = mapped_center_point.point
            mapped_ellipse.confidence.confidence = ellipse.confidence.confidence
            radius = np.linalg.norm(
                [
                    mapped_center_point.point.x - mapped_side_point.point.x,
                    mapped_center_point.point.y - mapped_side_point.point.y,
                    mapped_center_point.point.z - mapped_side_point.point.z,
                ]
            )
            mapped_ellipse.diameter = 2 * radius
            ellipses_3d.append(mapped_ellipse)
        except NoIntersectionError:
            logger.warn(
                'Got an ellipse with center point ({},{}) I could not transform.'.format(
                    ellipse.center.x,
                    ellipse.center.x),
                throttle_duration_sec=5)
    return ellipses_3d
