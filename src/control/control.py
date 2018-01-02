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
from m4atx_battery_monitor.msg import PowerReading
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

        # Subscribers
        rospy.Subscriber("/xiroi/battery_status_m4atx", PowerReading, self.control_battery)
        rospy.Subscriber("/xiroi/sensors/imu", Imu, self.control_imu)
        rospy.Subscriber("/xiroi/sensors/gps", NavSatFix, self.control_gps)
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.control_navsts)
        rospy.Subscriber("joy",Joy,self.control_joy,queue_size = 4)

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
        self.imu_init=True
        print "imu on"
       
    def control_gps(self,data_gps):
        self.gps_init=True
        print "gps on"
        
    def control_navsts(self, data_NavSts):
        self.navsts_init=True
        print "navsts on"

    # def turbot_mission(self):



    def init_system(self):

        if(self.imu_init==True and self.gps_init==True and self.navsts_init==True and self.atx_init==True):
            print"----------------System ON-------------------"
        
            
    
if __name__ == '__main__':

    rospy.init_node('Control')
    Control()
    rospy.spin()