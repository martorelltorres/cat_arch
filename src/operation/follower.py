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

        # Initialize some parameters
        self.nav_sts = NavSts()
        self.xiroi_position = PoseStamped()
        # self.ned_latitude = rospy.get_param("/navigator/ned_origin_lat")
        # self.ned_longitude = rospy.get_param("/navigator/ned_origin_lon")
        
        self.turbot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.security_radius=10 #meters
        self.scale_factor=130
        self.constant_velocity=80

        # Subscribers
        rospy.Subscriber("/navigation/nav_sts", NavSts, self.NavSts_callback)               #Turbot surface position 
        rospy.Subscriber("/navigation/nav_sts_acoustic", NavSts, self.NavSts_callback)      #Turbot underwater position  
        rospy.Subscriber("/sensors/gps", NavSatFix, self.gps_callback)                      #Xiroi position
        rospy.Subscriber("/sensors/imu",Imu , self.xiroi_orientation)                       #Xiroi orientation
        self.turbot_pose_pub = rospy.Publisher("/turbot/pose", PoseStamped, queue_size = 1)

        self.ned_init = False
        self.imu_init = False
        self.gps_init = False

        # Publisher
        self.pub_thrusters_setpoints = rospy.Publisher('/setpoints',                        #Thruster velocity message
                                             Setpoints,
                                             queue_size = 2)
        # Message
        self.msg = Setpoints()
        self.msg.header.frame_id = 'xiroi'
        self.msg.setpoints = np.array([0.0, 0.0])
        
    def NavSts_callback(self,nav_sts):

        """ This is the callback for the navigation message """
        self.turbot_pose[0] = nav_sts.position.north - 176.2    #X 
        self.turbot_pose[1] = nav_sts.position.east - 376    #Y
        self.turbot_pose[2] = nav_sts.position.depth    #Z
        self.turbot_pose[3] = nav_sts.orientation.roll
        self.turbot_pose[4] = nav_sts.orientation.pitch
        self.turbot_pose[5] = nav_sts.orientation.yaw
        self.turbot_pose[6] = nav_sts.global_position.latitude
        self.turbot_pose[7] = nav_sts.global_position.longitude
        self.turbot_pose[8] = nav_sts.origin.latitude  #NED_latitude
        self.turbot_pose[9] = nav_sts.origin.longitude #NED_longitude
        self.ned = NED.NED(self.turbot_pose[8], self.turbot_pose[9], 0.0)
        if not self.ned_init:
            self.ned_init = True

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = nav_sts.position.north - 176
        msg.pose.position.y = nav_sts.position.east - 376
        msg.pose.position.z = nav_sts.position.depth
        msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(nav_sts.orientation.roll, nav_sts.orientation.pitch, nav_sts.orientation.yaw))
        # msg.pose.covariance = np.zeros([36], dtype=np.float32)
        # msg.pose.covariance[0] = 1.0
        # msg.pose.covariance[7] = 1.0
        # msg.pose.covariance[14] = 1.0
        self.turbot_pose_pub.publish(msg)
        
        self.check_security_radius()
        
    def xiroi_orientation(self,data):
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

        if not self.imu_init:
            self.imu_init = True
        
    def gps_callback(self,data):
        if self.ned_init:
            self.xiroi_position = self.ned.geodetic2ned([data.latitude, data.longitude, 0.0]) #X Y Z
            if not self.gps_init:
                self.gps_init = True
      
    def check_security_radius(self):
        if self.gps_init and self.imu_init:
            self.x_distance = self.turbot_pose[0] -self.xiroi_position[0]
            self.y_distance = self.turbot_pose[1] -self.xiroi_position[1]
            self.radius = sqrt((self.x_distance**2)+(self.y_distance**2)) 
            # print "radius "+ str(self.radius)
            # print "securiry_radius "+ str(self.security_radius)
            # print "x_distance "+str(self.x_distance)
            # print "y_distance "+str(self.y_distance)
            # print " "

            if (self.radius < self.security_radius):
                self.go_back()
                # print "go_back"
            else:
                self.follow()
        else:
            rospy.logwarn("Waiting for GPS and IMU to be received...")

    def go_back(self):
        aux = np.array([0.0, 0.0])
        aux[0] = 0
        aux[1] = 0
        self.msg.setpoints = aux.clip(min=0.0, max=1.0)
        self.msg.header.stamp = rospy.Time.now()
        self.pub_thrusters_setpoints.publish(self.msg)
        self.msg.header.seq = self.msg.header.seq + 1

    def follow(self):
        if self.gps_init and self.imu_init:
            alpha_ref =-atan2(self.y_distance,self.x_distance)
            # position_error = (sqrt(self.x_distance**2 + self.y_distance**2))*cos(atan(self.y_distance/self.x_distance)-self.xiroi_orientation_y)
            angle_error = (alpha_ref + self.yaw) 
            Vr = self.constant_velocity*(cos(angle_error)+sin(angle_error))
            Vl = self.constant_velocity*(cos(angle_error)-sin(angle_error))

            # print "alpha_ref "+str(alpha_ref)
            # print "angle error "+str(angle_error)

            # if Vl<0 and Vr<0:
            #     print ""
            #     print "NEGATIVEE"
            #     print angle_error
            #     print alpha_ref
            #     print""
            # print "VL "+str((Vl))
            # print "VR "+str((Vr))
            # print " "

            aux = np.array([0.0, 0.0])
            aux[0] = Vr/self.scale_factor
            aux[1] = Vl/self.scale_factor
            self.msg.setpoints = aux.clip(min=0.0, max=1.0)
            self.msg.header.stamp = rospy.Time.now()
            self.pub_thrusters_setpoints.publish(self.msg)
            self.msg.header.seq = self.msg.header.seq + 1
        else:
            rospy.logwarn("Waiting for GPS and IMU to be received...")
            
    
if __name__ == '__main__':

    rospy.init_node('Follower')
    Follower()
    rospy.spin()