import math
import rospy
import tf2_ros as tf2
from geometry_msgs.msg import PointStamped

from bitbots_head_behavior.actions.look_at import AbstractLookAt


class SearchRecentBall(AbstractLookAt):
    """
    This action looks at the last position the ball has been seen
    and starts searching it from this position on.
    """

    def __init__(self, dsd, blackboard, parameters=None):
        super(SearchRecentBall, self).__init__(dsd, blackboard, parameters)
        self._config = self.blackboard.config['search_recent_ball']

        self._pan_speed = self._config['pan_speed']
        self._tilt_speed = self._config['tilt_speed']

        # TODO param
        self._ball_time_out = rospy.Duration.from_sec(self._config['ball_search_time'] + 10)

        self._offset_pattern = self._config['offset_pattern']

        self._threshold = self.blackboard.config['position_reached_threshold']

        # Get the coresponding motor goals for the ball position
        self._recent_ball_motor_goals = self._get_head_goals_for_recent_ball()

        self.pop_flag = False

        # Check if a ball exists
        if self._recent_ball_motor_goals is None:
            rospy.loginfo("No ball seen. So we are not able to seaarch for it.", logger_name="search_recent_ball")
            self.pop_flag = True
            return

        # Check if the ball is too old
        if rospy.Time.now() - self.blackboard.world_model.ball_last_seen() > self._ball_time_out:
            rospy.loginfo("Ball is too old to search for it. Let's forget it.", logger_name="search_recent_ball")
            self.pop_flag = True
            return

        # Init pattern index
        self.index = 0

    def _get_head_goals_for_recent_ball(self):
        """
        Returns the head motor goals to look at the most recent ball position.

        :retruns tuple(head_pan, head_tilt): The head motor goals
        """
        # Check if Ball has been seen
        if not self.blackboard.world_model.ball_seen:
            return

         # Get last ball position
        point = self.blackboard.world_model.get_ball_stamped()

        # Transform the points reference frame to be the head
        try:
            point = self.blackboard.head_capsule.tf_buffer.transform(point, self.head_tf_frame, timeout=rospy.Duration(0.9))
        except tf2.LookupException as e:
            rospy.logwarn('The frame {} is not being published (LookupException)'.format(self.head_tf_frame))
            return
        except tf2.ConnectivityException as e:
            rospy.logwarn('The transforms {} and {} are not connected in the TF Tree (ConnectivityException)'.format(point.header.frame_id, self.head_tf_frame))
            return
        except tf2.ExtrapolationException as e:
            rospy.logwarn('The transform {} is currently not available (ExtrapolationException)'.format(self.head_tf_frame))
            return

        motor_goals = self.get_motor_goals_from_point(point.point)
        return motor_goals

    def perform(self, reevaluate=False):
        """
        Call look_at to look at the point which our world-model determines to be the ball

        :param reevaluate: No effect here
        """

        if self.pop_flag:
            return self.pop()

        # Exit action if pattern is finished
        if self.index >= len(self._offset_pattern):
            return self.pop()

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()

        # Add offset pattern to last ball position
        head_motor_goal_pan = self._recent_ball_motor_goals[0] + math.radians(self._offset_pattern[self.index][0])
        head_motor_goal_tilt = self._recent_ball_motor_goals[1] + math.radians(self._offset_pattern[self.index][1])

        #TODO add clipping otherwise the robot gets stuck at max positions

        self.blackboard.head_capsule.send_motor_goals(head_motor_goal_pan, head_motor_goal_tilt, pan_speed=self._pan_speed, tilt_speed=self._tilt_speed)

        # Distance between the current and the goal position
        distance = math.sqrt((current_head_pan - head_motor_goal_pan) ** 2 + (current_head_tilt - head_motor_goal_tilt) ** 2)


        # Increment index when position is reached
        if distance < math.radians(self._threshold):
            self.index += 1
