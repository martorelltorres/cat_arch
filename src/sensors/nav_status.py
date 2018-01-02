#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from sensor_msgs.msg import Joy, NavSatFix, Imu
from evologics_ros_sync.msg import EvologicsUsbllong
from geometry_msgs.msg import Vector3Stamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from ned_tools import NED,utils, utils_ros
from auv_msgs.msg import NavSts
from nav_msgs.msg import Odometry
from math import *
import tf
import numpy as np
import rosparam

class NavStatus:
    def __init__(self):

        #Setting parameters
        self.init_pub=False
        # Subscribers
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.NED_navSts) 
        rospy.Subscriber("odometry/filtered_map",Odometry,self.set_navSts)
        # Publishers
        self.xiroi_pose_pub = rospy.Publisher('navigation/nav_sts',                       
                                             NavSts,
                                             queue_size = 2)

    def NED_navSts(self,nav_sts):
        self.origin_latitude=nav_sts.origin.latitude
        self.origin_longitude=nav_sts.origin.longitude
        self.ned = NED.NED(nav_sts.origin.latitude, nav_sts.origin.longitude, 0.0)
        self.init_pub=True

    def x(self):
        self.latitude = rospy.get_param("navigator/ned_origin_lat")
        self.longitude = rospy.get_param("navigator/ned_origin_lon")
        
    def set_navSts(self,data):
        if self.init_pub==True:
            quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)

            msg = NavSts()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.global_position.latitude = self.origin_latitude
            msg.global_position.longitude = self.origin_longitude
            msg.position.north = data.pose.pose.position.x
            msg.position.east = data.pose.pose.position.y
            msg.position.depth = 0.0
            msg.altitude=0.0
            msg.body_velocity.x = data.twist.twist.linear.x
            msg.body_velocity.y = data.twist.twist.linear.y
            msg.body_velocity.z = data.twist.twist.linear.z
            msg.orientation.roll = euler[0]
            msg.orientation.pitch = euler[1]
            msg.orientation.yaw = euler[2]
            msg.origin.latitude=self.origin_latitude
            msg.origin.longitude=self.origin_longitude

            self.xiroi_pose_pub.publish(msg)
            msg.header.seq = msg.header.seq + 1

        elif self.init_pub==False:
            rospy.logwarn("Waiting for NED information")  

    # def get_config(self):
    #     """ Get config from param server """
    #     if rospy.has_param("vehicle_name"):  # This parameter is in dynamics yaml
    #         self.vehicle_name = rospy.get_param('vehicle_name')
    #     else:
    #         rospy.logfatal("[%s]: vehicle_name parameter not found", self.name)
    #         exit(0)  

    #     param_dict = {'latitude':"navigator/ned_origin_lat",
    #                   'longitude':"navigator/ned_origin_lon",
    #                   }

    #     if not utils_ros.getRosParams(self, param_dict, self.name):
    #         rospy.logfatal("[%s]: shutdown due to invalid config parameters!", self.name)
    #         exit(0)  
       
    
if __name__ == '__main__':

    rospy.init_node('nav_sts')
    NavStatus()
    rospy.spin()