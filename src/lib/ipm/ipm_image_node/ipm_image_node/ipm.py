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

import cv2
from cv_bridge import CvBridge
from ipm_library.exceptions import CameraInfoNotSetException
from ipm_library.ipm import IPM
from ipm_library.utils import create_horizontal_plane
import numpy as np
from numpy.lib import recfunctions as rfn
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
import tf2_ros as tf2

try:
    from bitbots_tf_buffer import Buffer, TransformListener
    fast_tf_buffer_available = True
except ImportError:
    from tf2_ros import Buffer, TransformListener
    fast_tf_buffer_available = False
    pass  # If bitbots_tf_buffer is not available, use the default tf2.Buffer

cv_bridge = CvBridge()


class IPMImageNode(Node):

    def __init__(self, context=None) -> None:
        super().__init__('ipm_image_node', context=context)
        # Declare params
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('type', 'mask')
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('use_distortion', False)

        # We need to create a tf buffer
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create an IPM instance
        self.ipm = IPM(self.tf_buffer, distortion=self.get_parameter('use_distortion').value)

        # Subscribe to camera info
        self.create_subscription(CameraInfo, 'camera_info', self.ipm.set_camera_info, 1)

        # Create publisher
        self.result_publisher = self.create_publisher(PointCloud2, 'projected_point_cloud', 1)

        # Create subsciber
        self.create_subscription(Image, 'input', self.map_message, 1)

    def map_message(self, msg: Image) -> PointCloud2:
        """
        Map a mask or complete rgb image as a pointcloud on the field plane.

        :param msg: Message containing a mask of pixels or complete
                    rgb image that should be projected
        :returns: The projected point cloud
        """
        # Get params
        scale = self.get_parameter('scale').value
        output_frame = self.get_parameter('output_frame').value

        # Get field plane
        field = create_horizontal_plane()
        if field is None:
            return

        # Convert subsampled image
        image = cv2.resize(
            cv_bridge.imgmsg_to_cv2(msg),
            (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

        # Check if we have a mask or full image
        image_type = self.get_parameter('type').value
        if image_type == 'mask':
            # Get indices for all non 0 pixels
            # (the pixels which should be displayed in the pointcloud)
            point_idx_tuple = np.where(image != 0)
        elif image_type == 'rgb_image':
            # Get indices for all pixels
            X, Y = np.meshgrid(np.arange(image.shape[1]), np.arange(image.shape[0]))
            point_idx_tuple = (Y.ravel(), X.ravel())
        else:
            self.get_logger().error(f"Unsupported image type '{image_type}'!")
            return

        # Restructure index tuple to a array
        point_idx_array = np.empty((point_idx_tuple[0].shape[0], 2))
        point_idx_array[:, 0] = point_idx_tuple[1] / scale
        point_idx_array[:, 1] = point_idx_tuple[0] / scale

        # Map points
        try:
            points_on_plane = self.ipm.map_points(
                        field,
                        point_idx_array,
                        msg.header.stamp,
                        plane_frame_id=output_frame,
                        output_frame_id=output_frame)[1]
        except CameraInfoNotSetException:
            self.get_logger().warn(
                'Inverse perspective mapping should be performed, '
                'but no camera info was recived yet!',
                throttle_duration_sec=5)
            return
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self.get_logger().warn(
                'Inverse perspective mapping should be performed, '
                f'but no transform was found: {e}',
                throttle_duration_sec=5)
            return

        # Define fields of the point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

        # Add rgb data to pointcloud
        if image_type == 'rgb_image':
            # Add additional field for rgb values
            fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
            # Add unused alpha channel,
            # because the casting of 4 x uint8 -> uint32 is easier with 4 channels
            rgba = cv2.cvtColor(image, cv2.COLOR_RGB2RGBA)
            # Recast 4 uint8 channels to one uint32 per pixel
            pixel_values = np.ndarray(
                shape=(image.shape[0] * image.shape[1], ),
                dtype=np.uint32,
                buffer=memoryview(rgba))
            # Create dtype for structured pointcloud numpy array
            new_dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            # Allocate new array with said dtype that has space
            # for the float coordinates and color values
            combined_points = np.empty(len(pixel_values), dtype=new_dtype)
            # Copy xyz coordinates into new array
            combined_points[['x', 'y', 'z']] = rfn.unstructured_to_structured(
                points_on_plane,
                names=('x', 'y', 'z')
            )
            # Copy rgb values (that are stored in a single uint32)
            combined_points['rgb'] = pixel_values
            # Use common name for final point cloud
            points_on_plane = combined_points

        # Build pointcloud
        pc = create_cloud(
            Header(
                stamp=msg.header.stamp,
                frame_id=output_frame
            ),
            fields,
            points_on_plane)

        self.result_publisher.publish(pc)


def main(args=None):
    rclpy.init(args=args)
    node = IPMImageNode()
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
        # Handle keyboard interrupt gracefully
        node.get_logger().info('Shutting down IPM Image Node...')
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.try_shutdown()
