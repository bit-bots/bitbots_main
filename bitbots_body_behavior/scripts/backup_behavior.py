#!/usr/bin/env python3
import rospy
import math
import time
import actionlib
from actionlib_msgs.msg import GoalStatus

import humanoid_league_msgs.msg
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
from humanoid_league_msgs.msg import BallRelative, GoalRelative, GameState, PlayAnimationGoal, PlayAnimationAction, HeadMode
from geometry_msgs.msg import Twist


class Behavior(object):
    def __init__(self):
        # Walk time after Penalty/Game start
        self.init_walking_time = 1
        # Threshold to determin if we rotate or walk a curve.
        self.rotation_threshold = math.radians(10)
        # Rotation speed
        self.walking_rotation_scalar = 0.16
        # Rotation speed if we turn around the ball to face the goal
        self.moonwalk_rotation = -0.35
        # Walking speeds
        self.walking_speed_forward = 0.06
        self.walking_speed_sideward = 0.06
        # Time to walk the last 0.5 meters torward the ball
        self.reach_ball_time = 3.0
        # INITS
        self.ball_position = (100,100)
        self.ball_distance = 100
        self.ball_angle = 0.0
        self.ball_age = 100 
        self.goal_angle = 1.0
        self.end_not_allowed_to_move = False
        self.goal_in_front = False
        self.allow_to_move = False
        # The maximum angle for a goal to be "in front" of the robot
        self.goal_kick_threshold = math.radians(20)
        # Kick the Ball (True) or run through it (False)
        self.kick_behavior = True
        # Turn torwards the goal
        self.goal_behavior = True
        
        rospy.init_node('backup_backup_behavior')

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        self.anim = Anim()

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

        self.pub_head_mode = rospy.Publisher(
            'head_mode',
            HeadMode,
            queue_size=1)

        self.pub_walking = rospy.Publisher(
            'cmd_vel',
            Twist,
            queue_size=1)

    def start(self):
        # Starts behavior
        self.behave()

    def behave(self):
        # Set head mode
        self.head_look_for_ball()
        # Stop the robot
        self.stopAtBegin()
        # Wait for game to start
        while self.allow_to_move == False and not rospy.is_shutdown():
            time.sleep(0.1)
            rospy.loginfo_throttle(5, "Waiting for game to start")
        # Start core behavior
        while not rospy.is_shutdown():
            # Walk while steering torwards the ball
            self.goToBall()
            # Chose what to do if we reached the ball (pre determind)
            if self.goal_behavior:
                # Center the goal and kick
                self.moonWalk()
            if self.kick_behavior:
                # Kick forwards
                self.kick()
            else:
                # Run against the ball
                self.runThroughBall()
            # wait til the next iteration starts
            time.sleep(1)

    def gamestate_cb(self, msg):
        # Walk in
        if msg.allowedToMove and not self.allow_to_move:
            rospy.loginfo("End: Freeze")
            time.sleep(1)
            # Inform the behavior about the state change
            self.end_not_allowed_to_move = True
        # Set if we are allowed to move
        self.allow_to_move = msg.allowedToMove 
    
    def goal_relative_cb(self, msg):
        # Get goal relative to base footprint
        goal_obj = PointStamped(msg.header, msg.center_direction)
        x, y = self.get_ball_position_uv(goal_obj)
        goal_position = (float(x), float(y))
        # Calculate the angle in which we are facing the goal
        self.goal_angle = math.atan2(float(goal_position[1]),float(goal_position[0]))
        # Determin if the goal is in front of us
        if abs(self.goal_angle) < self.goal_kick_threshold:
            self.goal_in_front = True
        else:
            self.goal_in_front = False

    def ball_relative_cb(self, msg):
        # Get bll relative to base footprint
        ball_obj = PointStamped(msg.header, msg.ball_relative)
        x, y = self.get_ball_position_uv(ball_obj)
        self.ball_position = (float(x), float(y))
        # Set ball distance
        self.ball_distance = math.sqrt(float(self.ball_position[0])**2 + float(self.ball_position[1])**2)
        # Calculate the angle in which we are facing the ball
        self.ball_angle = math.atan2(float(self.ball_position[1]),float(self.ball_position[0]))
        # Set the refresh time
        self.ball_age = time.time()


    def get_ball_position_uv(self, ball):
        # Transforms to the base_footprint
        try:
            ball = self.tf_buffer.transform(ball, 'base_footprint', timeout=rospy.Duration(0.3))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return None
        return ball.point.x, ball.point.y

    def stopAtBegin(self):
        # Stop robot
        time.sleep(1)
        self.walkingWalkSteeredForward(0.0, 0.0)

    def walkIn(self, walk_time):
        # Walk into the field after beeing started / beeing panalised
        rospy.loginfo("Walking into the field")
        # Speed up
        self.walkingWalkSteeredForward(self.walking_speed_forward * 0.5, 0, recurr=False)
        time.sleep(2)
        self.walkingWalkSteeredForward(self.walking_speed_forward, 0, recurr=False)
        rospy.loginfo("Max speed")
        time.sleep(walk_time)
        self.walkingWalkSteeredForward(0, 0, recurr=False)
        time.sleep(1)
    
    def goToBall(self):
        # Walk torward the ball til its 0.5 meters in front of us
        while self.ball_distance > 0.5 and not rospy.is_shutdown():
            # Turn left and search for balls
            self.searchBall()
            # Face the ball if you found one
            self.turnToBall()
            rospy.loginfo_throttle(1, "Walking to ball")
            # Walk torward the ball with small corrections
            self.walkingWalkSteeredForward(self.walking_speed_forward, self.ball_angle * self.walking_rotation_scalar)
            time.sleep(0.5)
        # The ball is 0.5 meters in front of us
        rospy.loginfo("Print reached ball")

    def searchBall(self):
        # Turn left til we see the ball
        while not self.ball_seen() and not rospy.is_shutdown():
            rospy.loginfo_throttle(2, "Searching for ball")
            self.walkingTurn(self.walking_rotation_scalar * math.pi)
            time.sleep(0.5)

    def turnToBall(self):
        # Save the ball angle
        ball_angle = self.ball_angle
        delta = abs(ball_angle)
        # Turn torwards the ball
        while delta > self.rotation_threshold and not rospy.is_shutdown():
            # Save the ball angle
            ball_angle = self.ball_angle
            delta = abs(ball_angle)
            rospy.loginfo_throttle(2, "Rotating to Ball....")
            # Determin if we turn left or right
            direction = ball_angle/(delta+0.0000001)
            # Turn torwards this goal with a fixed speed
            rotation = direction * self.walking_rotation_scalar * math.pi
            self.walkingTurn(rotation)
            time.sleep(0.5)
        rospy.loginfo("Right direction")

    def moonWalk(self):
        # Walk sidewards around the ball until the goal is in front of us
        # Set head mode so we see the goals
        self.head_look_forwards()
        # Turn torwards the goal
        counter = 0
        while not self.goal_in_front and counter < 300 and not rospy.is_shutdown():
            # Determin if we turn left or right
            direction = -(self.goal_angle/abs(self.goal_angle + 0.00001))
            # Walk sideward while turning yourself
            self.walkingWalkSteeredSidewards(direction * self.walking_speed_sideward, direction* self.moonwalk_rotation)
            counter += 1
            time.sleep(0.1)
        # Now we are facing a goal or got a timeout
        # Return to ball head  mode
        self.head_look_for_ball()
        rospy.loginfo("Goal Found")

    def head_look_forwards(self):
        # Set front head mode
        head_mode = HeadMode()
        head_mode.headMode = 7
        self.pub_head_mode.publish(head_mode)

    def head_look_for_ball(self):
        # Set ball search head mode
        head_mode = HeadMode()
        head_mode.headMode = 0
        self.pub_head_mode.publish(head_mode)


    def ball_seen(self):
        # Determin if we seen an ball in the last time
        return (time.time() - self.ball_age) < 1

    def stopWalking(self):
        # Stop walking
        walking_message = Twist()
        self.pub_walking.publish(walking_message)

    def walkingTurn(self, rate, recurr=True):
        # Send message to turn the robot
        # Stop if we are not allowed to move
        if not self.allow_to_move:
            self.stopWalking()
            return
        # Walk in insted
        if self.end_not_allowed_to_move and recurr:
            self.walkIn(self.init_walking_time)
            self.end_not_allowed_to_move = False
        # Send wanlkin message
        walking_message = Twist()
        walking_message.angular.z = rate
        self.pub_walking.publish(walking_message)

    def walkingWalkSteeredForward(self, speed, rotation, recurr=True):
        # Send message to walk (and turn)
        # Stop if we are not allowed to move
        if not self.allow_to_move:
            self.stopWalking()
            return
        # Walk in insted
        if self.end_not_allowed_to_move and recurr:
            self.walkIn(self.init_walking_time)
            self.end_not_allowed_to_move = False
        walking_message = Twist()
        walking_message.angular.z = rotation
        walking_message.linear.x = speed
        self.pub_walking.publish(walking_message)

    def walkingWalkSteeredSidewards(self, speed, rotation, recurr=True):
        # Send message to walk sidewards (and turn)
        # Stop if we are not allowed to move
        if not self.allow_to_move:
            self.stopWalking()
            return
        # Walk in insted
        if self.end_not_allowed_to_move and recurr:
            self.walkIn(self.init_walking_time)
            self.end_not_allowed_to_move = False
        walking_message = Twist()
        walking_message.angular.z = rotation
        walking_message.linear.y = speed
        self.pub_walking.publish(walking_message)
    
    def kick(self):
        # Kick the ball
        # Walk the last bit torwards the ball
        self.walkingWalkSteeredForward(self.walking_speed_forward,0)
        time.sleep(self.reach_ball_time)
        self.walkingWalkSteeredForward(0.0, 0.0)
        time.sleep(1)
        rospy.loginfo("Kick")
        # Check if we kick on the left or on the right
        if self.ball_position[1] > 0:
            rospy.loginfo("Left")
            # Choose animation
            animation = "kick_left"
        else:
            rospy.loginfo("Right")
            # Choose animation
            animation = "kick_right"
        try:
            # play animation
            self.anim.anim_run(anim=animation)
            self.anim = Anim()
        except:
            rospy.logerr("Animation error")
        rospy.loginfo("Finished kick")

    def runThroughBall(self):
        rospy.loginfo("Run through ball")
        # Walk against the ball
        self.walkingWalkSteeredForward(self.walking_speed_forward,0)
        time.sleep(self.reach_ball_time)
        self.walkingWalkSteeredForward(0.0, 0.0)
        time.sleep(2)
        # Go back and to check if we lost the ball
        self.walkingWalkSteeredForward(- 0.8 * self.walking_speed_forward,0)
        time.sleep(self.reach_ball_time)
        self.walkingWalkSteeredForward(0.0, 0.0)
        # Chill
        time.sleep(1)
    
class Anim(object):
    def __init__(self):
        pass

    def anim_run(self, anim=None):
        anim_client = actionlib.SimpleActionClient('animation', humanoid_league_msgs.msg.PlayAnimationAction)
        if anim is None:
            anim = rospy.get_param("~anim")
        if anim is None or anim == "":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        first_try = anim_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            anim_client.wait_for_server()
            rospy.logwarn("Animation server now running, hcm will go on.")
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = anim
        goal.hcm = False
        state = anim_client.send_goal_and_wait(goal)
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)

if __name__ == "__main__":
    behave = Behavior()
    behave.start()
