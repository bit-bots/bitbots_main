import rospy
import math
import time
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
from humanoid_league_msgs.msg import BallRelative, GoalRelative, GameState
from geometry_msgs.msg import Twist


class Behavior(object):
    def __init__(self):
        self.rotation_threshold = math.radians(10)
        self.walking_rotation_scalar = 0.08
        self.moonwalk_rotation = -0.1
        self.walking_speed_forward = 0.11
        self.walking_speed_sideward = 0.6
        self.reach_ball_time = 5.0
        self.ball_position = (100,100)
        self.ball_distance = 100
        self.ball_angle = 0.0
        self.ball_age = 100
        self.goal_angle = 0.1
        self.goal_kick_threshold = math.radians(10)
        self.goal_in_front = False
        self.one_ball_found = False
        self.allow_to_move = True
        rospy.init_node('backup_backup_behaviour', anonymous=True)

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        # Subscribe to 'ball_relative'-message
        self.ball_relative_msg = rospy.Subscriber(
            'ball_relative',
            BallRelative,
            self.ball_relative_cb,
            queue_size=1,
            tcp_nodelay=True)

        self.goal_relative_msg = rospy.Subscriber(
            'goal_relative',
            GoalRelative,
            self.goal_relative_cb,
            queue_size=1,
            tcp_nodelay=True)

        self.game_control_msg = rospy.Subscriber(
            'gamestate',
            GameState,
            self.gamestate_cb,
            queue_size=1,
            tcp_nodelay=True)

        self.pub_walking = rospy.Publisher(
            'cmd_vel',
            Twist,
            queue_size=1)

    def gamestate_cb(self, msg):
        self.allow_to_move = msg.allowedToMove 
    
    def goal_relative_cb(self, msg):
        goal_obj = PointStamped(msg.header, msg.center_direction)
        x, y = self.get_ball_position_uv(goal_obj)
        goal_position = (float(x), float(y))
        self.goal_angle = math.atan2(float(goal_position[1]),float(goal_position[0]))
        if abs(self.goal_angle) < self.goal_kick_threshold:
            self.goal_in_front = True
        else:
            self.goal_in_front = False

    def ball_relative_cb(self, msg):
        ball_obj = PointStamped(msg.header, msg.ball_relative)
        x, y = self.get_ball_position_uv(ball_obj)
        self.ball_position = (float(x), float(y))
        self.ball_distance = math.sqrt(float(self.ball_position[0])**2 + float(self.ball_position[1])**2)
        self.ball_angle = math.atan2(float(self.ball_position[1]),float(self.ball_position[0]))
        self.one_ball_found = True # TODO remove
        self.ball_age = time.time()


    def get_ball_position_uv(self, ball):
        try:
            ball = self.tf_buffer.transform(ball, 'base_footprint', timeout=rospy.Duration(0.3))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return None
        return ball.point.x, ball.point.y

    def start(self):
        self.behave()

    def behave(self):
        while not rospy.is_shutdown():
            self.searchBall()
            self.turnToBall()
            self.goToBall()
            self.moonWalk()
            self.kick()
            time.sleep(3)
    
    def goToBall(self):
        while self.ball_distance > 0.6 and not rospy.is_shutdown():
            self.searchBall()
            self.turnToBall()
            print("Walking to ball")
            self.walkingWalkSteeredForward(self.walking_speed_forward, self.ball_angle * self.walking_rotation_scalar)
            time.sleep(0.5)
        print("Print reached ball")

    def searchBall(self):
        while not self.ball_seen() and not rospy.is_shutdown():
            print("Searching for ball")
            self.walkingTurn(self.walking_rotation_scalar * math.pi)
            time.sleep(0.5)

    def turnToBall(self):
        ball_angle = self.ball_angle
        delta = abs(ball_angle)
        while delta > self.rotation_threshold and not rospy.is_shutdown():
            print("Rotating to Ball....")
            direction = ball_angle/delta
            rotation = direction * self.walking_rotation_scalar * math.pi
            self.walkingTurn(rotation)
            time.sleep(0.5)
            ball_angle = self.ball_angle
            delta = abs(ball_angle)
        print("Right direction")

    def moonWalk(self):
        while not self.goal_in_front and not rospy.is_shutdown():
            direction = -(self.goal_angle/abs(self.goal_angle))
            self.walkingWalkSteeredSidewards(direction * self.walking_speed_sideward, direction* self.moonwalk_rotation)
        print("Goal Found")

    def ball_seen(self):
        return (time.time() - self.ball_age) < 3

    def stopWalking(self):
        walking_message = Twist()
        self.pub_walking.publish(walking_message)

    def walkingTurn(self, rate):
        if not self.allow_to_move:
            self.stopWalking()
            return
        walking_message = Twist()
        walking_message.angular.z = rate
        self.pub_walking.publish(walking_message)

    def walkingWalkSteeredForward(self, speed, rotation):
        if not self.allow_to_move:
            self.stopWalking()
            return
        walking_message = Twist()
        walking_message.angular.z = rotation
        walking_message.linear.x = speed
        self.pub_walking.publish(walking_message)

    def walkingWalkSteeredSidewards(self, speed, rotation):
        if not self.allow_to_move:
            self.stopWalking()
            return
        walking_message = Twist()
        walking_message.angular.z = rotation
        walking_message.linear.y = speed
        self.pub_walking.publish(walking_message)
    
    def kick(self):
        self.walkingWalkSteeredForward(self.walking_speed_forward,0)
        time.sleep(self.reach_ball_time)
        self.walkingWalkSteeredForward(0,0)
        print("Kick")
        if self.ball_position[1] > 0:
            print("Left")
        else:
            print("Right")
        # TODO kick
    
if __name__ == "__main__":
    behave = Behavior()
    behave.start()
