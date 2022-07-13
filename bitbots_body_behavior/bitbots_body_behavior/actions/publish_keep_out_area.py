from dynamic_stack_decider import AbstractActionElement
from humanoid_league_msgs.msg import Strategy
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


class PublishKeepOutArea(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        ball_position = self.blackboard.world_model.get_ball_position_xy()

        # we need to create multiple points since we can not set the inflation for only this layer
        points_distance = 0.5
        # todo these positions could be improved. maybe use linspace
        obstacle_points = [[ball_position[0], ball_position[1], 0],
                           [ball_position[0] + points_distance, ball_position[1] + points_distance, 0],
                           [ball_position[0] - points_distance, ball_position[1] + points_distance, 0],
                           [ball_position[0] + points_distance, ball_position[1] - points_distance, 0],
                           [ball_position[0] - points_distance, ball_position[1] - points_distance, 0]]

        dummy_header = Header()
        dummy_header.stamp = self.blackboard.node.get_clock().now().to_msg()
        dummy_header.frame_id = self.blackboard.map_frame
        msg = create_cloud_xyz32(dummy_header, obstacle_points)
        self.blackboard.pathfinding.keep_out_area_pub.publish(msg)
        self.pop()
