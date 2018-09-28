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
from auv_msgs.msg import NavSts
from math import *
import tf
import numpy as np
import rosparam

class WaypointFollower:
    def __init__(self):
        # Initialize some parameters
        self.current_position = PoseStamped()
        self.waypoint_pose = PoseStamped()
        self.goal = PoseStamped()
        self.security_radius = 10 
        self.repulsion_radius = 7.5
        self.scale_factor = 130
        self.constant_velocity = 80
        self.depth_threshold = 3
        self.max_mission_setpoint = rospy.get_param('max_mission_setpoint')
        self.abort_mission = False
        # Message
        self.thruster_setpoints = Setpoints()
        self.thruster_setpoints.header.frame_id = 'xiroi'
        self.thruster_setpoints.requester = 'operation'
        self.thruster_setpoints.priority = 30
        self.thruster_setpoints.setpoints = np.array([0.0, 0.0])

    def abort_mission (self):
        self.abort_mission = True

    def update_waypoint_position(self, position):
        self.waypoint_pose = position

    def update_current_pose(self,data):
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        self.current_yaw = euler[2]
        self.current_position_x = data.pose.position.x
        self.current_position_y = data.pose.position.y

    def get_setpoints(self):
        self.run()
        self.thruster_setpoints.header.stamp = rospy.Time.now()
        self.thruster_setpoints.header.seq = self.thruster_setpoints.header.seq + 1
        return self.thruster_setpoints

    def update_thrusters(self, current, goal):
        self.update_current_pose(current)
        self.update_waypoint_position(goal)
        return self.get_setpoints()
        
    def run(self):
        if not self.abort_mission:
            self.x_distance = self.waypoint_pose.pose.position.x -self.current_position_x
            self.y_distance = self.waypoint_pose.pose.position.y -self.current_position_y
            self.radius = sqrt((self.x_distance**2)+(self.y_distance**2)) 

            if (self.radius <= self.security_radius and self.radius>self.repulsion_radius):
                self.stopped()
            elif (self.radius <= self.repulsion_radius and self.waypoint_pose.pose.position.z<self.depth_threshold):
                self.repulsion()
            else:
                self.follow()

    def stopped(self):
        aux = np.array([0.0, 0.0])
        aux[0] = 0
        aux[1] = 0
        self.thruster_setpoints.setpoints = aux.clip(min=0.0, max=self.max_mission_setpoint)        

    def repulsion(self):
        alpha_ref =-atan2(self.y_distance,self.x_distance)
        angle_error = (alpha_ref + self.current_yaw) 
        Vr = self.constant_velocity*(cos(angle_error)+sin(angle_error))
        Vl = self.constant_velocity*(cos(angle_error)-sin(angle_error))
        aux = np.array([0.0, 0.0])
        aux[0] = -Vr/self.scale_factor
        aux[1] = -Vl/self.scale_factor
        self.thruster_setpoints.setpoints = aux.clip(min=-self.max_mission_setpoint, max=self.max_mission_setpoint)

    def follow(self):
        alpha_ref =-atan2(self.y_distance,self.x_distance)
        angle_error = (alpha_ref + self.current_yaw) 
        Vr = self.constant_velocity*(cos(angle_error)+sin(angle_error))
        Vl = self.constant_velocity*(cos(angle_error)-sin(angle_error))
        aux = np.array([0.0, 0.0])
        aux[0] = Vr/self.scale_factor 
        aux[1] = Vl/self.scale_factor
        if aux[0] < 0 and aux[1] < 0:
          aux[0] = aux[0]*-1
          aux[1] = aux[1]*-1
        self.thruster_setpoints.setpoints = aux.clip(min=-self.max_mission_setpoint, max=self.max_mission_setpoint)
