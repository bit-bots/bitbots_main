"""
CostmapCapsule
^^^^^^^^^^^^^^^^^^

Provides information about the cost of different positions and moves.
"""
import math
from typing import TYPE_CHECKING, List, Optional, Tuple

if TYPE_CHECKING:
    from bitbots_blackboard.blackboard import BodyBlackboard

import numpy as np
import tf2_ros as tf2
from bitbots_utils.utils import get_parameter_dict, get_parameters_from_other_node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid
from PIL import Image, ImageDraw
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from soccer_vision_3d_msgs.msg import Robot, RobotArray
from tf2_geometry_msgs import PointStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class CostmapCapsule:
    def __init__(self, blackboard: "BodyBlackboard"):
        self._blackboard = blackboard
        self._node = blackboard.node
        self.body_config = get_parameter_dict(self._blackboard.node, "body")

        self.map_frame: str = self._blackboard.node.get_parameter("map_frame").value
        self.base_footprint_frame: str = self._blackboard.node.get_parameter("base_footprint_frame").value

        parameters = get_parameters_from_other_node(
            self._blackboard.node, "/parameter_blackboard", ["field_length", "field_width", "goal_width"]
        )
        self.field_length: float = parameters["field_length"]
        self.field_width: float = parameters["field_width"]
        self.goal_width: float = parameters["goal_width"]
        self.map_margin: float = self.body_config["map_margin"]
        self.obstacle_costmap_smoothing_sigma: float = self.body_config["obstacle_costmap_smoothing_sigma"]
        self.obstacle_cost: float = self.body_config["obstacle_cost"]

        # Publisher for visualization in RViZ
        self.costmap_publisher = self._blackboard.node.create_publisher(OccupancyGrid, "debug/costmap", 1)

        self.base_costmap: Optional[np.ndarray] = None  # generated once in constructor field features
        self.costmap: Optional[np.ndarray] = None  # updated on the fly based on the base_costmap
        self.gradient_map: Optional[np.ndarray] = None  # global heading map (static) only dependent on field structure

        # Calculates the base costmap and gradient map based on it
        self.calc_base_costmap()
        self.calc_gradients()

    def robot_callback(self, msg: RobotArray):
        """
        Callback with new robot detections
        """
        # Init a new obstacle costmap
        obstacle_map = np.zeros_like(self.costmap)
        # Iterate over all robots
        robot: Robot
        for robot in msg.robots:
            # Convert position to array index
            idx_x, idx_y = self.field_2_costmap_coord(robot.bb.center.position.x, robot.bb.center.position.y)
            # TODO inflate
            # Draw obstacle with smoothing independent weight on obstacle costmap
            obstacle_map[idx_x, idx_y] = self.obstacle_cost * self.obstacle_costmap_smoothing_sigma
        # Smooth obstacle map
        obstacle_map = gaussian_filter(obstacle_map, self.obstacle_costmap_smoothing_sigma)
        # Get pass offsets
        # NOTE: as we currently do not use the kick, passing is not possible
        # self.pass_map = self.get_pass_regions()
        # Merge costmaps
        self.costmap = self.base_costmap + obstacle_map  # - self.pass_map  # NOTE: see above
        # Publish debug costmap
        self.publish_costmap()

    def publish_costmap(self):
        """
        Publishes the costmap for rviz
        """
        # Normalize costmap to match the rviz color scheme in a good way
        normalized_costmap = (
            (255 - ((self.costmap - np.min(self.costmap)) / (np.max(self.costmap) - np.min(self.costmap))) * 255 / 2.1)
            .astype(np.int8)
            .T
        )
        # Build the OccupancyGrid message
        msg: OccupancyGrid = msgify(
            OccupancyGrid,
            normalized_costmap,
            info=MapMetaData(
                resolution=0.1,
                origin=Pose(
                    position=Point(
                        x=-self.field_length / 2 - self.map_margin,
                        y=-self.field_width / 2 - self.map_margin,
                    )
                ),
            ),
        )
        # Change the frame to allow namespaces
        msg.header.frame_id = self.map_frame
        # Publish
        self.costmap_publisher.publish(msg)

    def get_pass_regions(self) -> np.ndarray:
        """
        Draws a costmap for the pass regions
        """
        pass_dist = 1.0
        pass_weight = 20.0
        pass_smooth = 4.0
        # Init a new costmap
        costmap = np.zeros_like(self.costmap)
        # Iterate over possible team mate poses
        for pose in self._blackboard.team_data.get_active_teammate_poses(count_goalies=False):
            # Get positions
            goal_position = np.array([self.field_length / 2, 0, 0])  # position of the opponent goal
            teammate_position = numpify(pose.position)
            # Get vector
            vector_teammate_to_goal = goal_position - numpify(pose.position)
            # Position between robot and goal but 1m away from the robot
            pass_pos = vector_teammate_to_goal / np.linalg.norm(vector_teammate_to_goal) * pass_dist + teammate_position
            # Convert position to array index
            idx_x, idx_y = self.field_2_costmap_coord(pass_pos[0], pass_pos[1])
            # Draw pass position with smoothing independent weight on costmap
            costmap[idx_x, idx_y] = pass_weight * pass_smooth
        # Smooth obstacle map
        return gaussian_filter(costmap, pass_smooth)

    def field_2_costmap_coord(self, x: float, y: float) -> Tuple[float, float]:
        """
        Converts a field position to the coresponding indices for the costmap.

        :param x: X Position relative to the center point. (Positive is towards the enemy goal)
        :param y: Y Position relative to the center point. (Positive is towards the left when we face the enemy goal)
        :return: The x index of the coresponding costmap slot, The y index of the coresponding costmap slot
        """
        idx_x = int(
            min(
                ((self.field_length + self.map_margin * 2) * 10) - 1,
                max(0, (x + self.field_length / 2 + self.map_margin) * 10),
            )
        )
        idx_y = int(
            min(
                ((self.field_width + self.map_margin * 2) * 10) - 1,
                max(0, (y + self.field_width / 2 + self.map_margin) * 10),
            )
        )
        return idx_x, idx_y

    def calc_gradients(self):
        """
        Recalculates the gradient map based on the current costmap.
        """
        gradient = np.gradient(self.base_costmap)
        norms = np.linalg.norm(gradient, axis=0)

        # normalize gradient length
        gradient = [np.where(norms == 0, 0, i / (norms + np.finfo(norms.dtype).eps)) for i in gradient]
        self.gradient_map = gradient

    def cost_at_relative_xy(self, x: float, y: float) -> float:
        """
        Returns cost at relative position to the base footprint.
        """
        if self.costmap is None:
            return 0.0

        point = PointStamped()
        point.header.stamp = Time(clock_type=ClockType.ROS_TIME).to_msg()
        point.header.frame_id = self.base_footprint_frame
        point.point.x = x
        point.point.y = y

        try:
            # Transform point of interest to the map
            point = self._blackboard.tf_buffer.transform(point, self.map_frame, timeout=Duration(seconds=0.3))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(str(e))
            return 0.0

        return self.get_cost_at_field_position(point.point.x, point.point.y)

    def calc_base_costmap(self):
        """
        Builds the base costmap based on the bahavior parameters.
        This costmap includes a gradient towards the enemy goal and high costs outside the playable area
        """
        # Get parameters
        goalpost_safety_distance: float = self.body_config[
            "goalpost_safety_distance"
        ]  # offset in y direction from the goalpost
        keep_out_border: float = self.body_config["keep_out_border"]  # dangerous border area
        in_field_value_our_side: float = self.body_config["in_field_value_our_side"]  # start value on our side
        corner_value: float = self.body_config["corner_value"]  # cost in a corner
        goalpost_value: float = self.body_config["goalpost_value"]  # cost at a goalpost
        goal_value: float = self.body_config["goal_value"]  # cost in the goal

        # Create Grid
        grid_x, grid_y = np.mgrid[
            0 : self.field_length + self.map_margin * 2 : (self.field_length + self.map_margin * 2) * 10j,
            0 : self.field_width + self.map_margin * 2 : (self.field_width + self.map_margin * 2) * 10j,
        ]

        fix_points: List[Tuple[Tuple[float, float], float]] = []

        # Add base points
        fix_points.extend(
            [
                # Corner points of the map (including margin)
                ((-self.map_margin, -self.map_margin), corner_value + in_field_value_our_side),
                ((self.field_length + self.map_margin, -self.map_margin), corner_value + in_field_value_our_side),
                ((-self.map_margin, self.field_width + self.map_margin), corner_value + in_field_value_our_side),
                (
                    (self.field_length + self.map_margin, self.field_width + self.map_margin),
                    corner_value + in_field_value_our_side,
                ),
                # Corner points of the field
                ((0, 0), corner_value + in_field_value_our_side),
                ((self.field_length, 0), corner_value),
                ((0, self.field_width), corner_value + in_field_value_our_side),
                ((self.field_length, self.field_width), corner_value),
                # Points in the field that pull the gradient down, so we don't play always in the middle
                ((keep_out_border, keep_out_border), in_field_value_our_side),
                ((keep_out_border, self.field_width - keep_out_border), in_field_value_our_side),
            ]
        )

        # Add goal area (including the dangerous parts on the side of the goal)
        fix_points.extend(
            [
                ((self.field_length, self.field_width / 2 - self.goal_width / 2), goalpost_value),
                ((self.field_length, self.field_width / 2 + self.goal_width / 2), goalpost_value),
                (
                    (self.field_length, self.field_width / 2 - self.goal_width / 2 + goalpost_safety_distance),
                    goal_value,
                ),
                (
                    (self.field_length, self.field_width / 2 + self.goal_width / 2 - goalpost_safety_distance),
                    goal_value,
                ),
                (
                    (
                        self.field_length + self.map_margin,
                        self.field_width / 2 - self.goal_width / 2 - goalpost_safety_distance,
                    ),
                    -0.2,
                ),
                (
                    (
                        self.field_length + self.map_margin,
                        self.field_width / 2 + self.goal_width / 2 + goalpost_safety_distance,
                    ),
                    -0.2,
                ),
            ]
        )

        # Apply map margin to fixpoints
        fix_points = [((p[0][0] + self.map_margin, p[0][1] + self.map_margin), p[1]) for p in fix_points]

        # Interpolate the keypoints from above to form the costmap
        interpolated = griddata(
            [p[0] for p in fix_points], [p[1] for p in fix_points], (grid_x, grid_y), method="linear"
        )

        # Smooth the costmap to get more continus gradients
        self.base_costmap = gaussian_filter(interpolated, self.body_config["base_costmap_smoothing_sigma"])
        self.costmap = self.base_costmap.copy()

    def get_gradient_at_field_position(self, x: float, y: float) -> Tuple[float, float]:
        """
        Gets the gradient tuple at a given field position
        :param x: Field coordiante in the x direction
        :param y: Field coordiante in the y direction
        """
        idx_x, idx_y = self.field_2_costmap_coord(x, y)
        return -self.gradient_map[0][idx_x, idx_y], -self.gradient_map[1][idx_x, idx_y]

    def get_cost_at_field_position(self, x: float, y: float) -> float:
        """
        Gets the costmap value at a given field position
        :param x: Field coordinate in the x direction
        :param y: Field coordinate in the y direction
        """
        idx_x, idx_y = self.field_2_costmap_coord(x, y)
        return self.costmap[idx_x, idx_y]

    def get_gradient_direction_at_field_position(self, x: float, y: float):
        """
        Returns the gradient direction at the given position
        :param x: Field coordiante in the x direction
        :param y: Field coordiante in the y direction
        """
        # for debugging only
        # if False and self.costmap.sum() > 0:
        #    # Create Grid
        #    grid_x, grid_y = np.mgrid[0:self.field_length:self.field_length * 10j,
        #                     0:self.field_width:self.field_width * 10j]
        #    plt.imshow(self.costmap.T, origin='lower')
        #    plt.show()
        #    plt.quiver(grid_x, grid_y, -self.gradient_map[0], -self.gradient_map[1])
        #    plt.show()

        grad = self.get_gradient_at_field_position(x, y)
        return math.atan2(grad[1], grad[0])

    def get_cost_of_kick_relative(self, x: float, y: float, direction: float, kick_length: float, angular_range: float):
        """
        Returns the cost of a kick at the given position and direction in base footprint frame
        :param x: Field coordiante in the x direction
        :param y: Field coordiante in the y direction
        :param direction: The direction of the kick
        :param kick_length: The length of the kick
        :param angular_range: The angular range of the kick"""
        if self.costmap is None:
            return 0.0

        pose = PoseStamped()
        pose.header.stamp = Time(clock_type=ClockType.ROS_TIME).to_msg()
        pose.header.frame_id = self.base_footprint_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        x, y, z, w = quaternion_from_euler(0, 0, direction)
        pose.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        try:
            # Transform point of interest to the map
            pose = self._blackboard.tf_buffer.transform(pose, self.map_frame, timeout=Duration(seconds=0.3))

        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._blackboard.node.get_logger().warn(str(e))
            return 0.0
        d = euler_from_quaternion(numpify(pose.pose.orientation))[2]
        return self.get_cost_of_kick(pose.pose.position.x, pose.pose.position.y, d, kick_length, angular_range)

    def get_cost_of_kick(self, x: float, y: float, direction: float, kick_length: float, angular_range: float) -> float:
        """
        Returns the cost of the kick at the given position
        :param x: Field coordiante in the x direction
        :param y: Field coordiante in the y direction
        :param direction: The direction of the kick
        :param kick_length: The length of the kick
        :param angular_range: The angular range of the kick
        """

        # create a mask in the size of the costmap consisting of 8-bit values initialized as 0
        mask = Image.new("L", (self.costmap.shape[1], self.costmap.shape[0]))

        # draw kick area on mask with ones
        maskd = ImageDraw.Draw(mask)
        # axes are switched in pillow

        b, a = self.field_2_costmap_coord(x, y)
        k = kick_length * 10
        m = a + k * math.sin(direction + 0.5 * angular_range)
        n = b + k * math.sin(0.5 * math.pi - (direction + 0.5 * angular_range))
        o = a + k * math.sin(direction - 0.5 * angular_range)
        p = b + k * math.sin(0.5 * math.pi - (direction - 0.5 * angular_range))
        maskd.polygon(((a, b), (m, n), (o, p)), fill=1)

        mask_array = np.array(mask)

        masked_costmap = self.costmap * mask_array

        # plt.imshow(self.costmap, origin='lower')
        # plt.show()
        # plt.imshow(masked_costmap, origin='lower')
        # plt.show()

        # The main influence should be the maximum cost in the area which is covered by the kick. This could be the field boundary, robots, ...
        # But we also want prio directions with lower min cost. This could be the goal area or the pass accept area of an teammate
        # This should contribute way less than the max and should have an impact if the max values are similar in all directions.
        return masked_costmap.max() * 0.75 + masked_costmap.min() * 0.25

    def get_current_cost_of_kick(self, direction: float, kick_length: float, angular_range: float):
        """
        Returns the cost of the kick at the current position
        :param direction: The direction of the kick
        :param kick_length: The length of the kick
        :param angular_range: The angular range of the kick
        """
        return self.get_cost_of_kick_relative(0, 0, direction, kick_length, angular_range)

    def get_best_kick_direction(
        self, min_angle: float, max_angle: float, num_kick_angles: int, kick_length: float, angular_range: float
    ) -> float:
        """
        Returns the best kick direction in the given range
        :param min_angle: The minimum angle of the kick
        :param max_angle: The maximum angle of the kick
        :param num_kick_angles: The number of angles to check
        :param kick_length: The length of the kick
        :param angular_range: The angular range of the kick
        """
        # list of possible kick directions, sorted by absolute value to
        # prefer forward kicks to side kicks if their costs are equal
        kick_directions = sorted(np.linspace(min_angle, max_angle, num=num_kick_angles), key=abs)

        # get the kick direction with the least cost
        kick_direction = kick_directions[
            np.argmin(
                [
                    self.get_current_cost_of_kick(
                        direction=direction, kick_length=kick_length, angular_range=angular_range
                    )
                    for direction in kick_directions
                ]
            )
        ]
        return kick_direction
