#!/usr/bin/env python2.7
import rospy
import tf2_ros
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan




class AmclNomotionUpdate(object):

    def __init__(self):
        rospy.init_node('amcl_nomotion_update')
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('/pre_scan', LaserScan, self.nomotion_update, queue_size=1, tcp_nodelay=True)
        self.pubMotion = rospy.Publisher('/scan', LaserScan, queue_size=1, tcp_nodelay=True)

        #transform tolerance
        self.tolerance_body = 0.1 #todo find magic value
        self.tolerance_head = 0.1

        self.rate = rospy.Rate(10)
        rospy.sleep(3) #wait to have old data to compare with
        rospy.spin()

    def nomotion_update(self, data):
        now = rospy.Time.now()
        past = now - rospy.Duration(1) #TODO:good value?

        if self.check_nomotion(now, past): #check if feet did not move
            if self.check_head_motion(now, past): #check if head moved
                self.pubMotion.publish(data)
                rospy.wait_for_service('request_nomotion_update')
                try: #call service
                    rospy.logwarn("Requesting nomotion_update")
                    nomotion_update = rospy.ServiceProxy('request_nomotion_update', Empty)
                    nomotion_update()
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed: ", e)
        else:
            rospy.logerr("moving")
        self.rate.sleep()


    def check_nomotion(self, now, past):
        try:
            #rospy.logwarn("check_nomotion")
            trans_feet_now = self.tf_buffer.lookup_transform('base_link', 'l_foot', now, rospy.Duration(1.0)).transform  # oder odom??
            trans_feet_past = self.tf_buffer.lookup_transform('base_link', 'l_foot', past, rospy.Duration(1.0)).transform

            if self.check_in_tolerance(trans_feet_now, trans_feet_past, self.tolerance_body):
                rospy.logwarn_throttle(10, "nomotion")
                return True
            else:
                rospy.logwarn_throttle(10,"feet moved")
                return False
        except tf2_ros.LookupException:
            rospy.logwarn_throttle(10,"AMCL: Could not transform from base_link to l_foot")
            return False
        except tf2_ros.ExtrapolationException:
            rospy.logwarn_throttle(10,"AMCL: Waiting for transforms to become available...")
            return False

    def check_head_motion(self, now, past):
        try:
            #rospy.logwarn("check_head_motion")
            trans_head_now = self.tf_buffer.lookup_transform('base_link', 'camera', now,rospy.Duration(1.0)).transform
            trans_head_past = self.tf_buffer.lookup_transform('base_link', 'camera', past,rospy.Duration(1.0)).transform

            if not self.check_in_tolerance(trans_head_now, trans_head_past, self.tolerance_head):
                rospy.logwarn_throttle(10,"new_head_position")
                return True
            else:
                rospy.logwarn_throttle(10,"head didnt move")
                return False
        except tf2_ros.LookupException:
            rospy.logwarn_throttle(10,"AMCL: Could not transform from base_link to head")
            return False
        except tf2_ros.ExtrapolationException:
            rospy.logwarn_throttle(10,"AMCL: Waiting for transforms to become available...")
            return False

    def check_in_tolerance(self, transformNew, transformOld, tolerance):
        # check if new transform is in tolerance area of old transform

        if abs(transformNew.translation.x - transformOld.translation.x) < tolerance and \
            abs(transformNew.translation.y - transformOld.translation.y) < tolerance and \
            abs(transformNew.translation.z - transformOld.translation.z) < tolerance and \
            abs(transformNew.rotation.x - transformOld.rotation.x) < tolerance and \
            abs(transformNew.rotation.y - transformOld.rotation.y) < tolerance and \
            abs(transformNew.rotation.z - transformOld.rotation.z) < tolerance and \
            abs(transformNew.rotation.w - transformOld.rotation.w) < tolerance:
            return True
        else:
            return False









if __name__ == "__main__":

    try:
        AmclNomotionUpdate()
    except rospy.ROSInterruptException:
        pass