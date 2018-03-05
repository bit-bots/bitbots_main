#!/usr/bin/env python3.5
import rospy
import time
from sensor_msgs.msg import Imu

class SubTest():

    def __init__(self):
        rospy.init_node("test_sub")
        self.arrt = []
        self.arrn = []
        self.sum =0
        self.count=0
        self.max = 0
        self.sub = rospy.Subscriber("test", Imu, self.cb, queue_size=1)
        self.f = open("latencies", 'w')

        while not rospy.is_shutdown():
            time.sleep(1)
        if self.count !=0:
            print("mean: " + str((self.sum/self.count)*1000))
        print("max: " + str(self.max*1000))

        i = 0
        for n in self.arrn:
            self.f.write(str(n) + "," + str(self.arrt[i]*1000) + "\n")
            i+=1
        self.f.close()

    def cb(self, msg:Imu):
        diff = rospy.get_time() - msg.header.stamp.to_sec()
        self.arrt.append(diff)
        self.arrn.append(msg.header.seq)
        self.sum += diff
        self.count +=1
        self.max = max(self.max, diff)





if __name__ == "__main__":
    SubTest()
