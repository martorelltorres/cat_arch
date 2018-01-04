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
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
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

        self.keep_position_enabled = False
        self.teleoperation_enabled = True

        self.current = PoseStamped()
        self.waypoint = PoseStamped()

        self.waypoint.header.frame_id = 'map'
        self.current.header.frame_id = 'map'

        # Services
        self.recovery_srv       =       rospy.Service('control/abort_mission',
                                                        RecoveryAction,
                                                        self.follower.abort_mission)
        self.enable_keep_position_srv = rospy.Service('control/enable_keep_position',
                                                        Empty,
                                                        self.enable_keep_position)
        self.disable_keep_position_srv = rospy.Service('control/disable_keep_position',
                                                        Empty,
                                                        self.disable_keep_position)
        self.enable_teleoperation_srv = rospy.Service('control/enable_teleoperation',
                                                        Empty,
                                                        self.enable_teleoperation)
        self.disable_teleoperation_srv = rospy.Service('control/disable_teleoperation',
                                                        Empty,
                                                        self.disable_teleoperation)
        # Publishers
        self.pub_thrusters_setpoints = rospy.Publisher('setpoints', Setpoints, queue_size = 2)
        self.turbot_pose_pub = rospy.Publisher("turbot/pose",PoseStamped,queue_size = 1)
        self.markerPub = rospy.Publisher('robotMarker',Marker,queue_size=1)

        # Subscribers
        rospy.Subscriber("setpoints_req", Setpoints, self.setpoints_req)           #Turbot surface position
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.waypoint_callback)           #Turbot surface position
        rospy.Subscriber("/navigation/nav_sts_acoustic", NavSts, self.waypoint_callback)  #Turbot underwater position
        rospy.Subscriber("odometry/filtered_map",Odometry,self.current_pose_callback)     #Current position and orientation

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
        if not self.keep_position_enabled:
            self.robotMarker.scale.x = 19.0
            self.robotMarker.scale.y = 19.0
            self.waypoint.header.stamp = rospy.Time.now()
        else:
            self.robotMarker.scale.x = 5.0
            self.robotMarker.scale.y = 5.0
        self.robotMarker.scale.z = 0.1
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 0.3
        self.markerPub.publish(self.robotMarker)

        self.turbot_pose_pub.publish(self.waypoint)

    def setpoints_req(self, msg):
        if self.teleoperation_enabled:
            self.pub_thrusters_setpoints.publish(msg)

    def waypoint_callback(self, msg):
        if self.keep_position_enabled:
            return
        # Convert from NED class to Pose class
        self.waypoint.header.stamp = rospy.Time.now()
        self.waypoint.pose.position.x = msg.position.north
        self.waypoint.pose.position.y = msg.position.east
        self.waypoint.pose.position.z = msg.position.depth
        self.waypoint.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw))



    def current_pose_callback(self, msg):
        self.current.header.stamp = rospy.Time.now()
        self.current.pose.position.x = msg.pose.pose.position.x
        self.current.pose.position.y = msg.pose.pose.position.y
        self.current.pose.position.z = msg.pose.pose.position.z
        self.current.pose.orientation = msg.pose.pose.orientation

        # Compute thruster setpoints
        self.thruster_setpoints = self.follower.update_thrusters(self.current, self.waypoint)
        self.draw_marker()

        if not self.teleoperation_enabled:
            self.pub_thrusters_setpoints.publish(self.thruster_setpoints)


    def enable_keep_position(self, req):
        print "ENABLE KEEP POSITION"
        self.keep_position_enabled = True
        self.teleoperation_enabled = False

        self.waypoint = copy.deepcopy(self.current)
        self.follower.security_radius = 2.5
        self.follower.repulsion_radius = 0.01

        return EmptyResponse()

    def disable_keep_position(self, req):
        print "DISABLE KEEP POSITION"
        self.keep_position_enabled = False
        self.teleoperation_enabled = True

        self.follower.security_radius = 10.0
        self.follower.repulsion_radius = 7.5

        return EmptyResponse()

    def disable_teleoperation(self, req):
        print "DISABLE TELEOPERATION"
        self.teleoperation_enabled = False
        self.enable_keep_position(EmptyRequest())
        return EmptyResponse()

    def enable_teleoperation(self, req):
        print "ENABLE TELEOPERATION"
        self.teleoperation_enabled = True
        self.disable_keep_position(EmptyRequest())
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('WaypointFollowerNode')
    WaypointFollowerNode()
    rospy.spin()