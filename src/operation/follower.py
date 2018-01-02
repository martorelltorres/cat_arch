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
from std_srvs.srv import Empty, EmptyResponse
from auv_msgs.msg import NavSts
from math import *
import tf
import numpy as np
import rosparam
import copy
from WaypointFollower.WaypointFollower import WaypointFollower


class WaypointFollowerNode:
    def __init__(self):
        self.follower = WaypointFollower()
        self.first_time = True
        self.keep_position = False
        self.current = PoseStamped()
        self.waypoint = PoseStamped()
        self.stop_publish=False

        # Subscribers
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.waypoint_callback)           #Turbot surface position 
        rospy.Subscriber("/navigation/nav_sts_acoustic", NavSts, self.waypoint_callback)  #Turbot underwater position  
        rospy.Subscriber("odometry/filtered_map",Odometry,self.current_pose_callback)     #Current position and orientation
        # Publishers
        self.pub_thrusters_setpoints = rospy.Publisher('setpoints', Setpoints, queue_size = 2)
        self.turbot_pose_pub = rospy.Publisher("turbot/pose",PoseStamped,queue_size = 1)
        self.markerPub = rospy.Publisher('robotMarker',Marker,queue_size=1)
        # Services
        self.recovery_srv       =       rospy.Service('/xiroi/control/abort_mission',
                                                        RecoveryAction,
                                                        self.follower.abort_mission)
        self.enable_keep_position_srv = rospy.Service('/xiroi/control/enable_keep_position',
                                                        Empty,
                                                        self.enable_keep_position)
        self.disable_keep_position_srv = rospy.Service('/xiroi/control/disable_keep_position',
                                                        Empty,
                                                        self.disable_keep_position)
        self.enable_teleoperation_srv = rospy.Service('/xiroi/control/enable_teleoperation',
                                                        Empty,
                                                        self.enable_teleoperation)
        self.disable_teleoperation_srv = rospy.Service('/xiroi/control/disable_teleoperation',
                                                        Empty,
                                                        self.disable_teleoperation)

    def draw_marker(self):
        #Security radius marker
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "map"
        self.robotMarker.header.stamp    = rospy.Time.now()
        self.robotMarker.ns = "robot"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.CYLINDER
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position = self.waypoint.pose.position
        self.robotMarker.pose.position.z = self.waypoint.pose.position.z
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        if not self.keep_position:
            self.robotMarker.scale.x = 19.0
            self.robotMarker.scale.y = 19.0
        else:
            self.robotMarker.scale.x = 5.0
            self.robotMarker.scale.y = 5.0
        self.robotMarker.scale.z = 0.1
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 0.3
        self.markerPub.publish(self.robotMarker)

    def waypoint_callback(self, msg):
        if self.keep_position:
            return
        # Convert from NED class to Pose class
        self.waypoint.header.stamp = rospy.Time.now()
        self.waypoint.header.frame_id = 'map'
        self.waypoint.pose.position.x = msg.position.north- 176.2
        self.waypoint.pose.position.y = msg.position.east- 376
        self.waypoint.pose.position.z = msg.position.depth
        self.waypoint.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw))
        
        self.turbot_pose_pub.publish(self.waypoint)        

    def current_pose_callback(self, msg):
        self.current.header.stamp = rospy.Time.now()
        self.current.header.frame_id = 'map'
        self.current.pose.position.x = msg.pose.pose.position.x
        self.current.pose.position.y = msg.pose.pose.position.y
        self.current.pose.position.z = msg.pose.pose.position.z
        self.current.pose.orientation = msg.pose.pose.orientation

        # Compute thruster setpoints 
        self.thruster_setpoints = self.follower.update_thrusters(self.current, self.waypoint)
        self.draw_marker()

        if not self.stop_publish:
            self.pub_thrusters_setpoints.publish(self.thruster_setpoints)
       

    def enable_keep_position(self, req):
        print "ENABLE KEEP POSITION"
        self.keep_position = True
        self.waypoint = copy.deepcopy(self.current)
        self.follower.security_radius = 2.5
        self.follower.repulsion_radius = 0.01
        return EmptyResponse()

    def disable_keep_position(self, req):
        print "DISABLE KEEP POSITION"
        self.keep_position = False
        self.follower.security_radius = 10.0
        self.follower.repulsion_radius = 7.5
        return EmptyResponse()

    def disable_teleoperation(self, req):
        print "DISABLE TELEOPERATION"
        self.stop_publish=False
        return EmptyResponse()

    def enable_teleoperation(self, req):
        print "ENABLE TELEOPERATION"
        self.stop_publish=True
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('WaypointFollowerNode')
    WaypointFollowerNode()
    rospy.spin()