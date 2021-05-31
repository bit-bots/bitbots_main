import time

import rospy


class Rate:

    def __init__(self, rate):
        self.current_time = rospy.Time()
        self.rate = rospy.Rate(rate)

    def sleep(self):
        """
        Makes sure that we dont run multiple times when in simulation due to /clock being slower that rate.
        """
        while True:
            self.rate.sleep()
            last_time = self.current_time
            self.current_time = rospy.Time.now()
            if last_time != self.current_time:
                # time actually changed
                return
            else:
                # we don't want to be busy waiting
                time.sleep(0.0001)
