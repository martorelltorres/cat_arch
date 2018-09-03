#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from geometry_msgs.msg import Vector3Stamped, PoseStamped,PoseWithCovariance,PoseWithCovarianceStamped,Quaternion,Point
from ned_tools import NED
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
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

	self.setpoint_timeout = rospy.Duration(0.2)
	self.setpoints = Setpoints()
	self.setpoints.setpoints = np.array([0.0, 0.0])
        self.keep_position_enabled = False
        self.teleoperation_enabled = True

        self.current = PoseStamped()
        self.waypoint = PoseStamped()

        self.waypoint.header.frame_id = 'map'
        self.current.header.frame_id = 'map'

        # Services
        self.recovery_srv       =       rospy.Service('control/abort_mission',
                                                        Empty,
                                                        self.follower_abort_mission)
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
        self.thrusters_setpoints_pub = rospy.Publisher('setpoints', Setpoints, queue_size = 1)
        self.goal_pub = rospy.Publisher("/goal_position",PoseStamped,queue_size = 1)
        self.marker_pub = rospy.Publisher('robotMarker', Marker, queue_size=1)

        # Subscribers
        rospy.Subscriber("setpoints_req", Setpoints, self.setpoints_req)                  
        rospy.Subscriber("/goal", NavSts, self.waypoint_callback)                         #Goal
        rospy.Subscriber("odometry/filtered_map",Odometry,self.current_pose_callback)     #Current position and orientation
        rospy.Timer(rospy.Duration(0.1), self.setpoint_timer)



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
        self.marker_pub.publish(self.robotMarker)

        self.goal_pub.publish(self.waypoint)

    def setpoint_timer(self, event):
    	if self.teleoperation_enabled:
    	    time_now = rospy.Time.now()
            if time_now - self.setpoints.header.stamp > self.setpoint_timeout:
    		  self.setpoints.setpoints = np.array([0.0, 0.0])
            self.thrusters_setpoints_pub.publish(self.setpoints)

    def setpoints_req(self, msg):
        if self.teleoperation_enabled:
	    self.setpoints = msg

    def waypoint_callback(self, msg):
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
        self.teleoperation_enabled=False
        if not self.teleoperation_enabled:
            self.thrusters_setpoints_pub.publish(self.thruster_setpoints)


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
        self.setpoints.setpoints = np.array([0.0, 0.0])
        self.setpoints.header.stamp = rospy.Time.now()

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

    def follower_abort_mission (self, req):
        print "ABORT MISSION"
        self.teleoperation_enabled = True

if __name__ == '__main__':
    rospy.init_node('WaypointFollowerNode')
    WaypointFollowerNode()
    rospy.spin()
