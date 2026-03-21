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

from ipm_library.ipm import IPM
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from soccer_ipm.msgs.ball import map_ball_array
from soccer_ipm.msgs.field_boundary import map_field_boundary
from soccer_ipm.msgs.goalpost import map_goalpost_array
from soccer_ipm.msgs.markings import map_marking_array
from soccer_ipm.msgs.obstacles import map_obstacle_array
from soccer_ipm.msgs.robots import map_robot_array
from soccer_ipm.utils import catch_camera_info_not_set_error, catch_tf_timing_error
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm

try:
    from bitbots_tf_buffer import Buffer, TransformListener
    fast_tf_buffer_available = True
except ImportError:
    from tf2_ros import Buffer, TransformListener
    fast_tf_buffer_available = False


class SoccerIPM(Node):

    def __init__(self) -> None:
        super().__init__('soccer_ipm')
        # Declare params
        self.declare_parameter('balls.ball_diameter', 0.153)
        self.declare_parameter('goalposts.footpoint_out_of_image_threshold', 0.8)
        self.declare_parameter('goalposts.object_default_dimensions.x', 0.1)
        self.declare_parameter('goalposts.object_default_dimensions.y', 0.1)
        self.declare_parameter('goalposts.object_default_dimensions.z', 1.0)
        self.declare_parameter('obstacles.footpoint_out_of_image_threshold', 0.8)
        self.declare_parameter('obstacles.object_default_dimensions.x', 0.2)
        self.declare_parameter('obstacles.object_default_dimensions.y', 0.2)
        self.declare_parameter('obstacles.object_default_dimensions.z', 1.0)
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('robots.footpoint_out_of_image_threshold', 0.8)
        self.declare_parameter('robots.object_default_dimensions.x', 0.2)
        self.declare_parameter('robots.object_default_dimensions.y', 0.2)
        self.declare_parameter('robots.object_default_dimensions.z', 1.0)
        self.declare_parameter('use_distortion', False)

        # We need to create a tf buffer
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create an IPM instance
        self.ipm = IPM(self.tf_buffer, distortion=self.get_parameter('use_distortion').value)

        # Subscribe to camera info
        self.create_subscription(CameraInfo, 'camera_info', self.ipm.set_camera_info, 1)

        # Create processing pipelines

        # Balls
        ball_publisher = self.create_publisher(sv3dm.BallArray, 'balls_relative', 1)

        @catch_camera_info_not_set_error(self.get_logger())
        @catch_tf_timing_error(self.get_logger())
        def ball_sub(msg):
            ball_publisher.publish(
                map_ball_array(
                    msg,
                    ipm=self.ipm,
                    output_frame=self.get_parameter('output_frame').value,
                    logger=self.get_logger(),
                    ball_diameter=self.get_parameter('balls.ball_diameter').value
                )
            )

        self.create_subscription(
            sv2dm.BallArray,
            'balls_in_image',
            ball_sub,
            1)

        # Goal posts
        goalpost_publisher = self.create_publisher(sv3dm.GoalpostArray, 'goal_posts_relative', 1)

        @catch_camera_info_not_set_error(self.get_logger())
        @catch_tf_timing_error(self.get_logger())
        def goalpost_pub(msg):
            goalpost_publisher.publish(
                map_goalpost_array(
                    msg,
                    ipm=self.ipm,
                    output_frame=self.get_parameter('output_frame').value,
                    logger=self.get_logger(),
                    footpoint_out_of_image_threshold=self.get_parameter(
                        'goalposts.footpoint_out_of_image_threshold').value,
                    object_default_dimensions=(
                        self.get_parameter('goalposts.object_default_dimensions.x').value,
                        self.get_parameter('goalposts.object_default_dimensions.y').value,
                        self.get_parameter('goalposts.object_default_dimensions.z').value
                    )
                )
            )

        self.create_subscription(
            sv2dm.GoalpostArray,
            'goal_posts_in_image',
            goalpost_pub,
            1)

        # Robots
        robot_publisher = self.create_publisher(sv3dm.RobotArray, 'robots_relative', 1)

        @catch_camera_info_not_set_error(self.get_logger())
        @catch_tf_timing_error(self.get_logger())
        def robot_sub(msg):
            robot_publisher.publish(
                map_robot_array(
                    msg,
                    ipm=self.ipm,
                    output_frame=self.get_parameter('output_frame').value,
                    logger=self.get_logger(),
                    footpoint_out_of_image_threshold=self.get_parameter(
                        'robots.footpoint_out_of_image_threshold').value,
                    object_default_dimensions=(
                        self.get_parameter('robots.object_default_dimensions.x').value,
                        self.get_parameter('robots.object_default_dimensions.y').value,
                        self.get_parameter('robots.object_default_dimensions.z').value
                    )
                )
            )

        self.create_subscription(
            sv2dm.RobotArray,
            'robots_in_image',
            robot_sub,
            1)

        # Obstacles
        obstacle_publisher = self.create_publisher(sv3dm.ObstacleArray, 'obstacles_relative', 1)

        @catch_camera_info_not_set_error(self.get_logger())
        @catch_tf_timing_error(self.get_logger())
        def obstacle_sub(msg):
            obstacle_publisher.publish(
                map_obstacle_array(
                    msg,
                    ipm=self.ipm,
                    output_frame=self.get_parameter('output_frame').value,
                    logger=self.get_logger(),
                    footpoint_out_of_image_threshold=self.get_parameter(
                        'obstacles.footpoint_out_of_image_threshold').value,
                    object_default_dimensions=(
                        self.get_parameter('obstacles.object_default_dimensions.x').value,
                        self.get_parameter('obstacles.object_default_dimensions.y').value,
                        self.get_parameter('obstacles.object_default_dimensions.z').value
                    )
                )
            )

        self.create_subscription(
            sv2dm.ObstacleArray,
            'obstacles_in_image',
            obstacle_sub,
            1)

        # Field boundary
        field_boundary_publisher = self.create_publisher(
            sv3dm.FieldBoundary,
            'field_boundary_relative',
            1)

        @catch_camera_info_not_set_error(self.get_logger())
        @catch_tf_timing_error(self.get_logger())
        def field_boundary_sub(msg):
            field_boundary_publisher.publish(
                map_field_boundary(
                    msg,
                    ipm=self.ipm,
                    output_frame=self.get_parameter('output_frame').value,
                    logger=self.get_logger()
                )
            )

        self.create_subscription(
            sv2dm.FieldBoundary,
            'field_boundary_in_image',
            field_boundary_sub,
            1)

        # Markings
        markings_publisher = self.create_publisher(sv3dm.MarkingArray, 'markings_relative', 1)

        @catch_camera_info_not_set_error(self.get_logger())
        @catch_tf_timing_error(self.get_logger())
        def markings_sub(msg):
            markings_publisher.publish(
                map_marking_array(
                    msg,
                    ipm=self.ipm,
                    output_frame=self.get_parameter('output_frame').value,
                    logger=self.get_logger()
                )
            )

        self.create_subscription(
            sv2dm.MarkingArray,
            'markings_in_image',
            markings_sub,
            1)


def main(args=None):
    rclpy.init(args=args)
    node = SoccerIPM()
    # Due to the fact that the bitbots_tf_buffer handles all tf2 communication in another node
    # we can use the single threaded EventsExecutor without running into deadlocks.
    if fast_tf_buffer_available:
        ex = EventsExecutor()
    else:
        ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
