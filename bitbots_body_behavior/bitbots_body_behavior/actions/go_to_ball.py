from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from actionlib_msgs.msg import GoalStatus
from rclpy.duration import Duration
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToBall, self).__init__(blackboard, dsd, parameters)

        if 'target' not in parameters.keys():
            self.blackboard.node.get_logger().error('The parameter "target" could not be used to decide whether map information is accesible')
        else:
            self.target = parameters['target']

        self.blocking = parameters.get('blocking', True)
        self.distance = parameters.get('distance', self.blackboard.config['ball_approach_dist'])

    def perform(self, reevaluate=False):

        pose_msg = self.blackboard.pathfinding.get_ball_goal(self.target, self.distance)
        self.blackboard.pathfinding.publish(pose_msg)

        approach_marker = Marker()
        approach_marker.pose.position.x = self.distance
        approach_marker.type = Marker.SPHERE
        approach_marker.action = Marker.MODIFY
        approach_marker.id = 1
        color = ColorRGBA()
        color.r = 1.0
        color.g = 1.0
        color.b = 1.0
        color.a = 1.0
        approach_marker.color = color
        approach_marker.lifetime = Duration(seconds=0.5).to_msg()
        scale = Vector3(0.2, 0.2, 0.2)
        approach_marker.scale = scale
        approach_marker.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        approach_marker.header.frame_id = self.blackboard.world_model.base_footprint_frame

        self.blackboard.pathfinding.approach_marker_pub.publish(approach_marker)

        if self.blackboard.pathfinding.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED] or not self.blocking:
            self.pop()
