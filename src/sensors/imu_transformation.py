#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from sensor_msgs.msg import Joy, NavSatFix, Imu
from evologics_ros_sync.msg import EvologicsUsbllong
from geometry_msgs.msg import Vector3Stamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from ned_tools import NED
from auv_msgs.msg import NavSts
from math import *
import tf
import numpy as np
import rosparam

class Follower:
    def __init__(self):

       
        # Subscribers
        rospy.Subscriber("/xiroi/sensors/imu",Imu , self.xiroi_orientation)                  #Xiroi orientation

        
    def xiroi_orientation(self,data):
        #this callback converts orientation quaternion in a orientation euler angles
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        self.yaw = euler[2]

        print "roll: "+str(roll)
        print "pitch: "+str(pitch)
        print "yaw: "+str(self.yaw)
        print " "
      
    
if __name__ == '__main__':

    rospy.init_node('Follower')
    Follower()
    rospy.spin()