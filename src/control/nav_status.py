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
        # Subscribers
        rospy.Subscriber("odometry/filtered_map",Odometry,self.odometry_cb)

        self.origin_latitude = rospy.get_param("navigator/ned_origin_lat")
        self.origin_longitude = rospy.get_param("navigator/ned_origin_lon")
        self.ned = NED.NED(self.origin_latitude, self.origin_longitude, 0.0)

        # Publishers
        self.xiroi_pose_pub = rospy.Publisher('navigation/nav_sts',
                                             NavSts,
                                             queue_size = 2)

    def odometry_cb(self, odom):
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        msg = NavSts()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        llh = self.ned.ned2geodetic([odom.pose.pose.position.x, odom.pose.pose.position.y, 0])

        msg.global_position.latitude = llh[0]
        msg.global_position.longitude = llh[1]
        msg.position.north = odom.pose.pose.position.x
        msg.position.east = odom.pose.pose.position.y
        msg.position.depth = 0.0
        msg.altitude=0.0
        msg.body_velocity.x = odom.twist.twist.linear.x
        msg.body_velocity.y = odom.twist.twist.linear.y
        msg.body_velocity.z = odom.twist.twist.linear.z
        msg.orientation.roll = euler[0]
        msg.orientation.pitch = euler[1]
        msg.orientation.yaw = euler[2]
        msg.origin.latitude=self.origin_latitude
        msg.origin.longitude=self.origin_longitude

        self.xiroi_pose_pub.publish(msg)
        msg.header.seq = msg.header.seq + 1

if __name__ == '__main__':

    rospy.init_node('nav_sts')
    NavStatus()
    rospy.spin()
