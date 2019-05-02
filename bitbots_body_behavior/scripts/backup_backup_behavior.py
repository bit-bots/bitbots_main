import rospy
import math
import time
from humanoid_league_msgs.msg import BallRelative
from geometry_msgs.msg import Twist

class Behavior(object):
    def __init__(self):
        self.rotation_threshold = math.radians(20)
        self.walking_rotation_scalar = 0.1
        self.walking_speed = 0.11
        self.ball_position = (100,100)
        self.ball_distance = 100
        self.ball_angle = 0.0
        self.one_ball_found = False
        rospy.init_node('backup_backup_behaviour', anonymous=True)
        # Subscribe to 'ball_relative'-message
        self.ball_relative_msg = rospy.Subscriber(
            'ball_relative',
            BallRelative,
            self.ball_relative_cb,
            queue_size=1,
            tcp_nodelay=True)

        self.pub_walking = rospy.Publisher(
            'cmd_vel',
            Twist,
            queue_size=1)

    def ball_relative_cb(self, msg):
        print("cb")
        self.ball_position = (float(msg.ball_relative.x), float(msg.ball_relative.y))
        self.ball_distance = math.sqrt(float(self.ball_position[0])**2 + float(self.ball_position[1])**2)
        if self.ball_position[0] == 0:
            if self.ball_position[1] > 0:
                self.ball_angle = 90
            else:
                self.ball_angle = -90
        else:
            self.ball_angle = math.atan(float(self.ball_position[1])/float(self.ball_position[0]))
        self.one_ball_found = True

    def start(self):
        while not self.one_ball_found:
            time.sleep(1)
            print("Wait for ball...")
        self.turnToBall()
        self.goToBall()

    def walkingTurn(self, rate):
        walking_message = Twist()
        walking_message.angular.z = rate
        self.pub_walking.publish(walking_message)

    def walkingWalkSteeredForward(self, speed, rotation):
        walking_message = Twist()
        walking_message.angular.z = rotation
        walking_message.linear.x = speed
        self.pub_walking.publish(walking_message)

    def turnToBall(self):
        ball_angle = self.ball_angle
        delta = abs(ball_angle)
        while delta > self.rotation_threshold:
            print(delta)
            print(math.degrees(self.ball_angle))
            print("Rotating to Ball....")
            rotation = -ball_angle * self.walking_rotation_scalar
            self.walkingTurn(rotation)
            time.sleep(0.1)
            ball_angle = ball_angle
            delta = abs(ball_angle)
        print("Right direction")
    
    def goToBall(self):
        while self.ball_distance > 0.2:
            print("Walking to ball")
            self.walkingWalkSteeredForward(self.walking_speed, -self.ball_angle * self.walking_rotation_scalar)
            time.sleep(0.5)
        print("Print reached ball")
    
if __name__ == "__main__":
    behave = Behavior()
    behave.start()
