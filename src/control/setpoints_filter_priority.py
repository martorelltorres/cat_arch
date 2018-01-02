#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from geometry_msgs.msg import Vector3Stamped, PoseStamped,PoseWithCovariance,PoseWithCovarianceStamped,Quaternion,Point
from ned_tools import NED
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from xiroi.srv import RecoveryAction
from std_srvs.srv import Empty
from auv_msgs.msg import NavSts
from math import *
import tf
import numpy as np
import rosparam
from WaypointFollower.WaypointFollower import WaypointFollower


class SetpointsFilterPriorityNode:
    def __init__(self):
        self.follower = WaypointFollower()
        self.first_time = True
        self.keep_position = False
        self.current = PoseStamped()
        self.waypoint = PoseStamped()

        # Subscriber
        rospy.Subscriber("setpoints", Setpoints, self.setpoints_filter_callback)           
        # Publisher
        self.pub__setpoints = rospy.Publisher('setpoints', Setpoints, queue_size = 2)
        

    def setpoints_filter_callback(self, msg):

        if(msg.priority=70):

        else:    
        

if __name__ == '__main__':
    rospy.init_node('SetpointsFilterPriorityNode')
    SetpointsFilterPriorityNode()
    rospy.spin()