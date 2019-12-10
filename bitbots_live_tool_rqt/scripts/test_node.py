#!/usr/bin/env python3


import rospy
from std_msgs.msg import Header

from humanoid_league_msgs.msg import BallRelative, ObstaclesRelative, ObstacleRelative, Strategy, GameState, RobotControlState
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose2D
import math
import yaml
import rospkg
import os
import tf



# Dictonary for roles
actionDecoder = {'ROLE_IDLING': 0, 'ROLE_OTHER': 1, 'ROLE_STRIKER': 2, 'ROLE_SUPPORTER': 3, 'ROLE_DEFENDER': 4, 'ROLE_GOALIE': 5 }


# Loads the dictonary of coordinates from pathmaker
def getCoordinates(filename):
    rp = rospkg.RosPack()
    fname = os.path.join(rp.get_path('bitbots_live_tool_rqt'), 'resource', 'paths', filename)

    with open(fname, "r") as file:
        positions = yaml.load(file) # da ist ein Dictonary drin

    file.close()
    return positions.get("positions")



def vec_rotate(x, y, angle_rad):
    xneu = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    yneu = y * math.cos(angle_rad) + x * math.sin(angle_rad)
    return [xneu, yneu]




def publisher_main():
    #initiieren des publishers
    rospy.init_node('publisher')
    print('started publisher node')
    pub = rospy.Publisher('ball_relative', BallRelative, queue_size = 10)
    pubRobo = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size = 10)
    pubTeam = rospy.Publisher('obstacles_relative', ObstaclesRelative, queue_size = 10)
    pubStrategy = rospy.Publisher('strategy', Strategy, queue_size = 10)
    pubGame = rospy.Publisher('gamestate', GameState, queue_size = 10)
    pubState = rospy.Publisher('robot_state', RobotControlState, queue_size = 10)
    pubTarget = rospy.Publisher('move_base_simple/goal', Pose2D, queue_size = 10)


    rate = rospy.Rate(10)

    timeCounter = 30
    roboActionCounter = 30
    firsthalf = True
    durationHalfGame = 60



    # Coordinates from pathMaker ========================================================================================

    # robo1 with pathmaker
    robo1 = getCoordinates("robo4.yaml")
    robo1Length = len(robo1)
    robo1Counter = 1


    # teammates with pathmaker
    teammate1 = getCoordinates('TeamClubMate1.yaml')
    team1Length = len(teammate1) #anzahl eintraege
    team1Counter = 1

    teammate2 = getCoordinates('TeamClubMate2.yaml')
    team2Length = len(teammate2)
    team2Counter = 1


    # opponents with pathmaker
    opponent1 = getCoordinates('SuperScaryOpponent.yaml')
    op1Length = len(opponent1)
    op1Counter = 1

    # opponents with pathmaker
    undef = getCoordinates('undef.yaml')
    undefLength = len(opponent1)
    undefCounter = 1

    # ball with pathmaker
    ball = getCoordinates('HeartBall.yaml')
    ballLength = len(ball)
    ballCounter = 1

    #teammate1[0 % length ].get('x') # fuer 0 ein counter, dann entsteht loop
    #teammate1[1].get('x') # an der ersten Stelle x-wert




    while not rospy.is_shutdown():

        # Ball with pathmaker
        msgBall = BallRelative()
        msgBall.header.stamp = rospy.Time.now()
        msgBall.header.frame_id = "base_link"
        msgBall.ball_relative.y = ball[ballCounter % ballLength].get('x')
        msgBall.ball_relative.x = ball[ballCounter % ballLength].get('y')
        msgBall.confidence = 1.0
        pub.publish(msgBall)
        ballCounter += 1



        # Robo1 with pathmaker
        msgRobo = PoseWithCovarianceStamped()
        msgRobo.header.stamp = rospy.Time.now()
        msgRobo.pose.pose.position.x = robo1[int(robo1Counter) % robo1Length].get('x')
        msgRobo.pose.pose.position.y = robo1[int(robo1Counter) % robo1Length].get('y')

        # Angle of robot in quaternions
        angle = robo1[int(robo1Counter) % robo1Length].get('ang')
        quaternion = tf.transformations.quaternion_from_euler(0, 0, float(angle))

        msgRobo.pose.pose.orientation.x = quaternion[0]
        msgRobo.pose.pose.orientation.y = quaternion[1]
        msgRobo.pose.pose.orientation.z = quaternion[2]
        msgRobo.pose.pose.orientation.w = quaternion[3]

        pubRobo.publish(msgRobo)


        # Role of Robo1, gets information from pathMaker
        msgStrategy = Strategy()
        msgRoleString = robo1[int(robo1Counter) % robo1Length].get('action')
        msgStrategy.role = actionDecoder.get(msgRoleString) #actiondecoder gleicht den string ab mit dictonary und gibt int zurueck

        # Action of Robo1, changes after short time (roboActionCounter)
        if roboActionCounter == 0:
            msgStrategy.action = 3 # TRYING_TO_SCORE
        else:
            msgStrategy.action = 2 # GOING_TO_BALL


        pubStrategy.publish(msgStrategy)

        roboActionCounter -= 1
        roboActionCounter = max(roboActionCounter, 0)
        robo1Counter += 1



        # Teammates with pathmaker, contains list of teammates
        msgTeam = ObstaclesRelative()
        msgTeam1 = ObstacleRelative()

        msgTeam1.color = 2  # magenta
        msgTeam1.position.x = teammate1[int(team1Counter) % team1Length].get('x')
        msgTeam1.position.y = teammate1[int(team1Counter) % team1Length].get('y')


        msgTeam2 = ObstacleRelative()
        msgTeam2.color = 2 # magenta
        msgTeam2.position.x = teammate2[int(team2Counter) % team2Length].get('x')
        msgTeam2.position.y = teammate2[int(team2Counter) % team2Length].get('y')


        # Opponents with pathmaker, contains list of opponents

        msgOp = ObstaclesRelative()

        msgUndef = ObstacleRelative()
        msgUndef.color = 1 # undef
        msgUndef.position.x = undef[int(undefCounter) % undefLength].get('x')
        msgUndef.position.y = undef[int(undefCounter) % undefLength].get('y')

        msgOp1 = ObstacleRelative()
        msgOp1.color = 3 # cyan
        msgOp1.position.x = opponent1[int(op1Counter) % op1Length].get('x')
        msgOp1.position.y = opponent1[int(op1Counter) % op1Length].get('y')



        # Publish all obstacles

        msgTeam.obstacles = [msgTeam1, msgTeam2, msgOp1, msgUndef]
        pubTeam.publish(msgTeam)
        team1Counter += 1
        team2Counter += 1
        op1Counter += 1
        undefCounter += 1


        # GameState msgs ===========================================================================================

        # Penalty: Seconds till unpenalized and boolean
        msgGame = GameState()
        msgBall.header.stamp = rospy.Time.now()
        msgGame.secondsTillUnpenalized = timeCounter
        # Penalty boolean
        msgGame.penalized = timeCounter > 0

        # Sets halftime and rest secs
        msgGame.firstHalf = firsthalf
        msgGame.secondsRemaining = durationHalfGame


        # Sets Score
        msgGame.ownScore = 7
        msgGame.rivalScore = 1


        # team colors
        msgGame.teamColor = 1 # magenta


        pubGame.publish(msgGame)
        timeCounter -= 1
        timeCounter = max(timeCounter, 0)
        durationHalfGame -= 1
        if durationHalfGame == 0:
            durationHalfGame = 60
            firsthalf = False


        # Sets hardware state
        msgState = RobotControlState()
        msgState.state = 10
        pubState.publish(msgState)



        # Target
        msgTarget = Pose2D()
        if firsthalf:
            msgTarget.x = 3.5
            msgTarget.y = 2.0
        else:
            msgTarget.x = 2.0
            msgTarget.y = 1.0
        pubTarget.publish(msgTarget)



        rate.sleep()




if __name__ == '__main__':
    try:
        publisher_main()
    except rospy.ROSInterruptException:
        pass


