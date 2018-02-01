#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

class CoMCalculator: 
    
    def __init__(self):
        rospy.init_node('com_berechnung', anonymous=True)
        self.Mass = 0
        #get robot description from URDF
        robot = URDF.from_parameter_server()
        self.links = robot.link_map
        
        #Delete links, which contain no mass description
        del self.links["base_link"]
        del self.links["L_CAMERA"]
        del self.links["L_Hand_L"]
        del self.links["L_Hand_R"]
        del self.links["L_FOOT_BOTTOM_L"]
        del self.links["L_FOOT_BOTTOM_R"]
        del self.links["L_FOOT_FRONT_L"]
        del self.links["L_FOOT_FRONT_R"]
        del self.links["L_IMU"]
        
        #Calculate the total mass of the robot
        for link in self.links:
            self.Mass += self.links[link].inertial.mass

        rospy.loginfo("Mass of robot is %f", self.Mass)
        self.calculator()
    
    def calculator(self):
        #initialisations for tf and marker
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        zuTransformieren = geometry_msgs.msg.PointStamped()
        transformiert = geometry_msgs.msg.PointStamped()
        x = 0
        y = 0
        z = 0
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        pub = rospy.Publisher('/com', Marker, queue_size=1)
        
        rate = rospy.Rate(1)
        rospy.sleep(1)
        
        #loop for calculating the CoM while robot is not shutdown
        while not rospy.is_shutdown():
            for link in self.links:
                try:
                    #get transformation matrix of link
                    trans = tfBuffer.lookup_transform("base_link", link, rospy.Time())
                    #transform CoM of link
                    zuTransformieren.point.x = self.links[link].inertial.origin.xyz[0]
                    zuTransformieren.point.y = self.links[link].inertial.origin.xyz[1]
                    zuTransformieren.point.z = self.links[link].inertial.origin.xyz[2]
                    zuTransformieren.header.frame_id = link
                    zuTransformieren.header.stamp = rospy.get_rostime()
                    transformiert = tf_geo.do_transform_point(zuTransformieren,trans)
                    #calculate part of CoM equation depending on link
                    x += self.links[link].inertial.mass * transformiert.point.x
                    y += self.links[link].inertial.mass * transformiert.point.y
                    z += self.links[link].inertial.mass * transformiert.point.z
                except tf2_ros.TransformException as err:
                    rospy.logerr("TF error in COM computation %s", err)

            '''except (tf2_ros.ConnectivityException):
                print "ConnectivityException"
            except (tf2_ros.LookupException):
                print "LookupException"
            except (tf2_ros.ExtrapolationException):
                print "ExtrapolationException"
            except (tf2_ros.InvalidArgumentException):
                print "InvalidArgumentException"'''
                
            #finish CoM calculation
            x = x/self.Mass
            y = y/self.Mass
            z = z/self.Mass

            #send CoM position to RViZ
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            pub.publish(marker)

            try:
                # catch exeption of moving backwarts in time, when restarting simulator
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
            
            
if __name__ == '__main__':
    calc = CoMCalculator()