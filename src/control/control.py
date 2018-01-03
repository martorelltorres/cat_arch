#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from geometry_msgs.msg import Vector3Stamped, PoseStamped,PoseWithCovariance,PoseWithCovarianceStamped,Quaternion,Point
from ned_tools import NED
from sensor_msgs.msg import Joy, NavSatFix, Imu, NavSatStatus
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
#from m4atx_battery_monitor.msg import PowerReading
from auv_msgs.msg import NavSts
from math import *
import tf
import numpy as np
import rosparam

class Control:
    def __init__(self):

        # Initialize some parameters
        self.full_charge = 12.65        # 100%
        self.mediumUP_charge = 12.45    # 75%
        self.mediumDOWN_charge = 12.24  # 50%
        self.low_charge = 12.06         # 25%

        self.init_soc_threshold=55

        self.imu_init = False
        self.gps_init = False
        self.navsts_init = False
        self.atx_init = False

        self.origin_latitude = rospy.get_param("navigator/ned_origin_lat")
        self.origin_longitude = rospy.get_param("navigator/ned_origin_lon")
        self.ned = NED.NED(self.origin_latitude, self.origin_longitude, 0.0)

        # Subscribers
        #rospy.Subscriber("battery_status_m4atx", PowerReading, self.control_battery)
        rospy.Subscriber("sensors/imu_raw", Imu, self.control_imu)
        rospy.Subscriber("sensors/gps_raw", NavSatFix, self.control_gps)
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.control_navsts)
        # rospy.Subscriber("joy", Joy, self.control_joy, queue_size = 4)

        # Publishers
        self.imu_pub = rospy.Publisher('sensors/imu', Imu, queue_size = 1)
        self.gps_pose_pub = rospy.Publisher('sensors/gps', PoseWithCovarianceStamped, queue_size = 1)

    def control_battery(self,data):

        self.read_input_voltage = data.volts_read_value[0]
        self.read_twelve_volts = data.volts_read_value[1]
        self.read_five_volts = data.volts_read_value[2]
        self.read_threepointthree_volts = data.volts_read_value[3]
        self.read_ignition_voltage = data.volts_read_value[4]

        self.full_input_voltage = data.volts_full_value[0]
        self.full_twelve_volts = data.volts_full_value[1]
        self.full_five_volts = data.volts_full_value[2]
        self.full_threepointthree_volts = data.volts_full_value[3]
        self.state_of_charge = data.input_soc

        if(self.state_of_charge>self.init_soc_threshold):
            self.atx_init = True
        else:
            print "stopping thrusters"

    def control_joy(self,joy):
        self.time= rospy.Time().now()
        self.message_time=joy.Header.header.stamp.secs


    def control_imu(self,data_imu):
        if not self.imu_init:
            rospy.loginfo('IMU is ON')
            self.imu_init=True
        self.imu_pub.publish(data_imu)

    def control_gps(self, data_gps):
        if not self.gps_init:
            rospy.loginfo('GPS is ON')
            self.gps_init=True

        llh = [data_gps.latitude, data_gps.longitude, data_gps.altitude]
        ned = self.ned.geodetic2ned(llh)

        msg = PoseWithCovarianceStamped();
        msg.header = data_gps.header
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = ned[1]
        msg.pose.pose.position.y = ned[0]
        msg.pose.pose.position.z = ned[2]
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = data_gps.position_covariance[0]
        msg.pose.covariance[7] = data_gps.position_covariance[4]
        msg.pose.covariance[14] = data_gps.position_covariance[8]

        self.gps_pose_pub.publish(msg)

    def control_navsts(self, data_NavSts):
        self.navsts_init=True
        print "navsts on"


    def init_system(self):
        if self.imu_init and self.gps_init and self.atx_init:
            print"----------------System ON-------------------"



if __name__ == '__main__':

    rospy.init_node('Control')
    Control()
    rospy.spin()