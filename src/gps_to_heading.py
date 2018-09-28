#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
import math

from ned_tools import NED,utils, utils_ros
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class GpsToHeading(object):
	def __init__(self):
		self.gps_timeout = rospy.get_param("gps_timeout")
		self.gps_min_speed = rospy.get_param("gps_min_speed")
		
		self.origin_latitude = rospy.get_param("navigator/ned_origin_lat")
		self.origin_longitude = rospy.get_param("navigator/ned_origin_lon")
		self.ned = NED.NED(self.origin_latitude, self.origin_longitude, 0.0)

		self.odom_msg = Odometry()
		self.gps_msg = NavSatFix()
		self.imu_msg = Imu()
		self.last_ned = []
		self.last_header = []
		self.first_iteration = True


		# Subscribers
		rospy.Subscriber("gps", NavSatFix, self.gps_cb)
		#rospy.Subscriber("imu", Imu, self.imu_cb)

		# Publishers
		self.odom_pub = rospy.Publisher('odom',
		                                Odometry,
		                                queue_size = 2)
		print "GPS to heading ready! Waiting for GPS messages..."

	#def imu_cb(self, imu_msg):
	#	self.imu_msg = imu_msg

	def gps_cb(self, gps_msg):
		print "GPS received"
		self.gps_msg = gps_msg
		ned_pose = self.ned.geodetic2ned([self.gps_msg.latitude, self.gps_msg.longitude, 0])

		if self.first_iteration:
			self.first_iteration = False
		else:
			delta_ned = ned_pose - self.last_ned
			delta_t = self.gps_msg.header.stamp.to_sec() - self.last_header.stamp.to_sec()
			speed_ned = delta_ned / delta_t
			speed = math.sqrt(speed_ned[0]*speed_ned[0]+speed_ned[1]*speed_ned[1])
			if speed > self.gps_min_speed and delta_t < self.gps_timeout:
				self.odom_msg.header = self.gps_msg.header
				yaw = math.atan2(delta_ned[1], delta_ned[0])
				quaternion = quaternion_from_euler(0, 0, yaw)
				self.odom_msg.pose.pose.orientation.x = quaternion[0]
				self.odom_msg.pose.pose.orientation.y = quaternion[1]
				self.odom_msg.pose.pose.orientation.z = quaternion[2]
				self.odom_msg.pose.pose.orientation.w = quaternion[3]
				self.odom_pub.publish(self.odom_msg)
			else:
				if speed <= self.gps_min_speed:
					print "Speed below threshold"
				elif delta_t >= self.gps_timeout:
					print "GPS timeout"

		self.last_ned = ned_pose
		self.last_header = self.gps_msg.header

if __name__ == '__main__':
    rospy.init_node('gps_to_heading')
    GpsToHeading()
    rospy.spin()
