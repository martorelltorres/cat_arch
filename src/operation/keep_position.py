#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from xiroi.msg import Setpoints
from geometry_msgs.msg import Vector3Stamped, PoseStamped,PoseWithCovariance,PoseWithCovarianceStamped,Quaternion,Point
from ned_tools import NED
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from auv_msgs.msg import NavSts
from math import *
import tf
import numpy as np
import rosparam

class keep_position:
    def __init__(self):

        # Initialize some parameters
        self.nav_sts = NavSts()
        self.xiroi_position = PoseStamped()
        self.turbot_pose = []
        self.security_radius=7
        self.scale_factor=130
        self.constant_velocity=80
        # self.depth_threshold=3
        self.ned_init = False
        self.imu_init = False
        self.gps_init = False

        # Subscribers
       rospy.Subscriber("/xiroi/sensors/gps",NavSatTransform,self.xiroi_pose,queue_size = 1)
     
        # Publishers
        self.pub_thrusters_setpoints = rospy.Publisher('setpoints',                        #Thruster velocity message
                                            Setpoints,
                                             queue_size = 2)
        self.turbot_pose_pub = rospy.Publisher("turbot/pose",
                                     PoseStamped,
                                      queue_size = 1)
        #Services
        self.enable_keep_position_srv = rospy.Service('/xiroi/control/enable_keep_position',
                                Empty,
                                self.enable_keep_position)
        self.disable_keep_position_srv = rospy.Service('/xiroi/control/disable_keep_position',
                                Empty,
                                self.disable_keep_position)
        # Message
        self.msg = Setpoints()
        self.msg.header.frame_id = 'xiroi'
        self.msg.setpoints = np.array([0.0, 0.0])

    def enable_keep_position (self,srv):
        self.check_keep_position()

    def disable_keep_position (self,srv):
        self.abort_mission_srv = rospy.ServiceProxy('/xiroi/control/abort_mission',Empty)

    def xiroi_pose(self,data):
        self.xiroi_position_x = data.pose.pose.position.x
        self.xiroi_position_y = data.pose.pose.position.y

        if not self.gps_init:
            self.gps_init = True
 
    def self.check_keep_position(self):
        if self.gps_init:
            self.x_distance = self.xiroi_position_x+1
            self.y_distance = self.xiroi_position_y+1
            self.radius = sqrt((self.x_distance**2)+(self.y_distance**2)) 

            if(self.radius <= self.threshold_radius):
                self.stopped()
            elif
                self.go_to_waypoint()
        else:
            rospy.loginfo_throttle(90,"Waiting for GPS to be received...")

    def stopped(self):
        aux = np.array([0.0, 0.0])
        aux[0] = 0
        aux[1] = 0
        self.msg.setpoints = aux.clip(min=0.0, max=1.0)
        self.msg.header.stamp = rospy.Time.now()
        self.pub_thrusters_setpoints.publish(self.msg)
        self.msg.header.seq = self.msg.header.seq + 1
    
    def go_to_waypoint(self):
        if self.gps_init and self.imu_init:
            angle_error =-atan2(self.y_distance,self.x_distance)
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

    rospy.init_node('keep_position')
    keep_position()
    rospy.spin()