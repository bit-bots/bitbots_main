#!/usr/bin/env python3.5

import rospy



class FakeDataGenerator():

    def __init__(self):
        rospy.init_node("fake_data_generator")

        # image space
        self.ball_image_pub
        self.ball_can_image_pub
        self.goal_part_image_pub
        self.goal_image_pub
        self.line_image_pub
        self.obstacle_image_pub

        # relative
        self.ball_rel_pub = rospy.Publisher("/ball_relative", BallRelative, queue_size=10)
        self.goal_rel_pub
        self.obstacle_rel_pub
        self.line_rel_pub

        # position
        self.position_pub

        # model
        self.personal_model_pub
        self.team_model_pub


        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.ball_rel_pub.publish(self.get_ball_rel_msg())


    def get_ball_rel_msg(self, x, y, c):

if __name__ == "__main__":
    gernerator = FakeDataGenerator()