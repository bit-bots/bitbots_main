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

from ipm_interfaces.srv import MapPoint, MapPointCloud2
from ipm_library.exceptions import InvalidPlaneException, NoIntersectionError
from ipm_library.ipm import IPM
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy

try:
    from bitbots_tf_buffer import Buffer, TransformListener
    fast_tf_buffer_available = True
except ImportError:
    from tf2_ros import Buffer, TransformListener
    fast_tf_buffer_available = False
    pass  # If bitbots_tf_buffer is not available, use the default tf2.Buffer


class IPMService(Node):

    def __init__(self, context=None) -> None:
        super().__init__('ipm_service', context=context)
        # Declare params
        self.declare_parameter('use_distortion', False)
        self.tf_buffer = Buffer(Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create ipm library instance
        self.ipm = IPM(self.tf_buffer, distortion=self.get_parameter('use_distortion').value)
        # Create subs
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera_info', self.ipm.set_camera_info, 1)
        # Create services
        self.point_srv = self.create_service(
            MapPoint, 'map_point', self.point_mapping_callback)
        self.point_cloud_srv = self.create_service(
            MapPointCloud2, 'map_pointcloud2', self.point_cloud_mapping_callback)

    def point_mapping_callback(
            self,
            request: MapPoint.Request,
            response: MapPoint.Response) -> MapPoint.Response:
        """
        Process the service request to map a given point.

        :param request: Service request
        :param response: Service response instance
        :returns: Filled out service response
        """
        # Check for camera info
        if not self.ipm.camera_info_received():
            response.result = MapPoint.Response.RESULT_NO_CAMERA_INFO
            return response

        # Map optional plane_frame_id from '' to None
        if request.plane_frame_id == '':
            plane_frame_id = None
        else:
            plane_frame_id = request.plane_frame_id

        # Map optional output_frame_id from '' to None
        if request.output_frame_id == '':
            output_frame_id = None
        else:
            output_frame_id = request.output_frame_id

        # Maps the given point and handle different result scenarios
        try:
            response.point = self.ipm.map_point(
                request.plane,
                request.point,
                request.time,
                plane_frame_id,
                output_frame_id)
            response.result = MapPoint.Response.RESULT_SUCCESS
        except NoIntersectionError:
            response.result = MapPoint.Response.RESULT_NO_INTERSECTION
        except InvalidPlaneException:
            response.result = MapPoint.Response.RESULT_INVALID_PLANE
        return response

    def point_cloud_mapping_callback(
            self,
            request: MapPointCloud2.Request,
            response: MapPointCloud2.Response) -> MapPointCloud2.Response:
        """
        Process the service request to map a given point cloud.

        Input points are defined in image space.

        :param request: Service request
        :param response: Service response instance
        :returns: Filled out service response
        """
        # Check for camera info
        if not self.ipm.camera_info_received():
            response.result = MapPointCloud2.Response.RESULT_NO_CAMERA_INFO
            return response

        # Map optional plane_frame_id from '' to None
        if request.plane_frame_id == '':
            plane_frame_id = None
        else:
            plane_frame_id = request.plane_frame_id

        # Map optional output_frame_id from '' to None
        if request.output_frame_id == '':
            output_frame_id = None
        else:
            output_frame_id = request.output_frame_id

        # Map the given point and handle different result scenarios
        try:
            header, mapped_points = self.ipm.map_points(
                request.plane,
                read_points_numpy(request.points),
                request.points.header.stamp,
                plane_frame_id,
                output_frame_id)

            # Convert them into a PointCloud2
            response.points = create_cloud_xyz32(header, mapped_points)

            response.result = MapPointCloud2.Response.RESULT_SUCCESS
        except InvalidPlaneException:
            response.result = MapPointCloud2.Response.RESULT_INVALID_PLANE

        return response


def main(args=None):
    rclpy.init(args=args)
    node = IPMService()
    if fast_tf_buffer_available:
        # If bitbots_tf_buffer is available, we can use the EventsExecutor
        # which is single threaded, using it without the decoupled bitbots_tf_buffer
        # would lead to deadlocks.
        ex = EventsExecutor()
    else:
        ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        # Spin the node to process incoming messages
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IPM service node.')
    finally:
        # Clean up the node
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
