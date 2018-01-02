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

class Follower:
    def __init__(self):

        # Initialize some parameters
        self.nav_sts = NavSts()
        self.xiroi_position = PoseStamped()
        self.turbot_pose = []
        self.security_radius=10 
        self.repulsion_radius=7.5
        self.scale_factor=130
        self.constant_velocity=80
        self.depth_threshold=3
        self.ned_init = False
        self.imu_init = False
        self.gps_init = False

        # Subscribers
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.NavSts_callback)           #Turbot surface position 
        rospy.Subscriber("/navigation/nav_sts_acoustic", NavSts, self.NavSts_callback)  #Turbot underwater position  
        rospy.Subscriber("odometry/filtered_map",Odometry,self.xiroi_pose)             #Xiroi position and orientation
     
        # Publishers
        self.pub_thrusters_setpoints = rospy.Publisher('setpoints',Setpoints,queue_size = 2)
        self.turbot_pose_pub = rospy.Publisher("turbot/pose",PoseStamped,queue_size = 1)
        self.markerPub = rospy.Publisher('robotMarker',Marker,queue_size=1)
        #Services
        self.recovery_srv = rospy.Service('control/abort_mission',RecoveryAction,self.abort_mission)
        # Message
        self.msg = Setpoints()
        self.msg.header.frame_id = 'xiroi'
        self.msg.setpoints = np.array([0.0, 0.0])

    def abort_mission (self,req):
        self.abort_mission = True
        #TODO:act consequently

    def NavSts_callback(self,nav_sts):

        """ This is the callback for the navigation message """
        #self.turbot_pose[2] = nav_sts.position.depth    #Z
        self.turbot_pose = nav_sts.position
        self.turbot_velocity = nav_sts.body_velocity

        ## APANYO FEO
        self.turbot_pose.north = nav_sts.position.north - 176.2    #X 
        self.turbot_pose.east = nav_sts.position.east - 376    #Y

        if not self.ned_init:
            self.ned = NED.NED(nav_sts.origin.latitude, nav_sts.origin.longitude, 0.0)
            self.ned_init = True

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.turbot_pose.north
        msg.pose.position.y = self.turbot_pose.east
        msg.pose.position.z = self.turbot_pose.depth
        msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(nav_sts.orientation.roll, nav_sts.orientation.pitch, nav_sts.orientation.yaw))
        self.turbot_pose_pub.publish(msg)

        #Security radius marker
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "map"
        self.robotMarker.header.stamp    = rospy.Time.now()
        self.robotMarker.ns = "robot"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.CYLINDER
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position = msg.pose.position
        self.robotMarker.pose.position.z = msg.pose.position.z 
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 19.0
        self.robotMarker.scale.y = 19.0
        self.robotMarker.scale.z = 0.1
        self.robotMarker.color.r = 1.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 0.3
        self.markerPub.publish(self.robotMarker)
                
        self.check_security_radius()
        
    def xiroi_pose(self,data):
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        self.yaw = euler[2]

        if not self.imu_init:
            self.imu_init = True

        if self.ned_init:
            self.xiroi_position_x = data.pose.pose.position.x
            self.xiroi_position_y = data.pose.pose.position.y

            if not self.gps_init:
                self.gps_init = True
        

    def check_security_radius(self):
        if self.gps_init and self.imu_init:
            self.x_distance = self.turbot_pose.north -self.xiroi_position_x
            self.y_distance = self.turbot_pose.east -self.xiroi_position_y
            self.radius = sqrt((self.x_distance**2)+(self.y_distance**2)) 
  
            if(self.radius <= self.security_radius and self.radius>self.repulsion_radius):
                self.stopped()
            elif(self.radius <= self.repulsion_radius and self.turbot_pose.depth<self.depth_threshold):
                self.repulsion()
            else:
                self.follow()
        else:
            rospy.logwarn_throttle(90, "Waiting for GPS and IMU to be received...")


    def stopped(self):
        aux = np.array([0.0, 0.0])
        aux[0] = 0
        aux[1] = 0
        self.msg.setpoints = aux.clip(min=0.0, max=1.0)
        self.msg.header.stamp = rospy.Time.now()
        self.pub_thrusters_setpoints.publish(self.msg)
        self.msg.header.seq = self.msg.header.seq + 1

    def repulsion(self):
        alpha_ref =-atan2(self.y_distance,self.x_distance)
        angle_error = (alpha_ref + self.yaw) 
        Vr = self.constant_velocity*(cos(angle_error)+sin(angle_error))
        Vl = self.constant_velocity*(cos(angle_error)-sin(angle_error))
        aux = np.array([0.0, 0.0])
        aux[0] = -Vr/self.scale_factor
        aux[1] = -Vl/self.scale_factor
        self.msg.setpoints = aux.clip(min=-1.0, max=1.0)
        self.msg.header.stamp = rospy.Time.now()
        self.pub_thrusters_setpoints.publish(self.msg)
        self.msg.header.seq = self.msg.header.seq + 1

    def follow(self):
        if self.gps_init and self.imu_init:
            alpha_ref =-atan2(self.y_distance,self.x_distance)
            angle_error = (alpha_ref + self.yaw) 
            Vr = self.constant_velocity*(cos(angle_error)+sin(angle_error))
            Vl = self.constant_velocity*(cos(angle_error)-sin(angle_error))
            aux = np.array([0.0, 0.0])
            aux[0] = Vr/self.scale_factor
            aux[1] = Vl/self.scale_factor
            self.msg.setpoints = aux.clip(min=-1.0, max=1.0)
            self.msg.header.stamp = rospy.Time.now()
            self.pub_thrusters_setpoints.publish(self.msg)
            self.msg.header.seq = self.msg.header.seq + 1
        else:
            rospy.logwarn("Waiting for GPS and IMU to be received...")

            
    
if __name__ == '__main__':

    rospy.init_node('Follower')
    Follower()
    rospy.spin()