#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from sensor_msgs.msg import Joy, NavSatFix, Imu
from evologics_ros_sync.msg import EvologicsUsbllong
from geometry_msgs.msg import PoseStamped
from m4atx_battery_monitor.msg import PowerReading
from xiroi.msg import BatteryInfo
from ned_tools import NED
from auv_msgs.msg import NavSts
from math import *
import numpy as np
import rosparam

class Safety:
    def __init__(self):

        # Initialize some parameters
        self.full_charge = 12.65        # 100%
        self.mediumUP_charge = 12.45    # 75%
        self.mediumDOWN_charge = 12.24  # 50%
        self.low_charge = 12.06         # 25%
        
              
        # Subscribers
        rospy.Subscriber("/battery_status_m4atx", PowerReading, self.battery_satus)

        #Publishers
        self.pub_battery_info=rospy.Publisher('/state_of_charge',BatteryInfo,queue_size=2)

    
    def battery_satus(self,data):
        
        self.read_input_voltage = data.volts_read_value[0]
        self.read_twelve_volts = data.volts_read_value[1]
        self.read_five_volts = data.volts_read_value[2]
        self.read_threepointthree_volts = data.volts_read_value[3]
        self.read_ignition_voltage = data.volts_read_value[4]

        self.full_input_voltage = data.volts_full_value[0]
        self.full_twelve_volts = data.volts_full_value[1]
        self.full_five_volts = data.volts_full_value[2]
        self.full_threepointthree_volts = data.volts_full_value[3]
        # self.full_ignition_voltage = data.volts_full_value[4]

        self.battery_temperature = data.temperature


        print self.read_input_voltage
        
        if(self.full_charge>=self.read_input_voltage>=self.mediumUP_charge):
            print "4 lights" 
            self.battery_info = 100
        elif(self.mediumUP_charge>=self.read_input_voltage>=self.mediumDOWN_charge):
            print "3 lights"
            self.battery_info = 50
        elif(self.mediumDOWN_charge>=self.read_input_voltage>=self.low_charge):
            print "2 lights"
            self.battery_info = 25
        elif(self.read_input_voltage>self.full_charge):
            print "over voltage"
            self.battery_info = 25
        elif(self.read_input_voltage<=self.low_charge):
            print "STOP"
        print self.battery_info
        self.pub_battery_info.publish(self.battery_info)
        
        
if __name__ == '__main__':

    rospy.init_node('Safety')
    Safety()
    rospy.spin()