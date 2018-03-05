#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('xiroi')
from geometry_msgs.msg import Vector3Stamped, PoseStamped,PoseWithCovariance,PoseWithCovarianceStamped,Quaternion,Point
from nav_msgs.msg import Odometry
from auv_msgs.msg import NavSts
from math import *

class WaypointPublisher:
  def __init__(self):
    rospy.init_node('waypoint_publisher')
    self.waypoints_x = rospy.get_param('waypoints_x')
    self.waypoints_y = rospy.get_param('waypoints_y')
    self.waypoint_tolerance = rospy.get_param('waypoint_tolerance')
    self.current = 0
    self.current_waypoint = [self.waypoints_x[self.current], self.waypoints_y[self.current]]
    print 'Total: ' + str(len(self.waypoints_x)) + ' waypoints!'
    print self.waypoints_x
    print self.waypoints_y

    self.nav_sts = NavSts()

    self.started = False

    self.waypoint_pub = rospy.Publisher("/navigation/nav_sts", NavSts,queue_size=1)
    rospy.Subscriber("odometry/filtered_map", Odometry, self.current_pose_callback)     #Current position and orientation
    rospy.Timer(rospy.Duration(0.5), self.update)
    rospy.spin()


  def current_pose_callback(self, msg):
    if not self.started:
      self.started = True
    self.current_pose = msg.pose.pose.position;

  def update(self, event):
    if not self.started:
      print('Waiting for Xiroi pose to be available...')
      return
    # Compute error
    error_x = self.current_pose.x - self.current_waypoint[0]
    error_y = self.current_pose.y - self.current_waypoint[1]
    error = sqrt(error_x*error_x+error_y*error_y)
    if error < self.waypoint_tolerance:
      if self.current < len(self.waypoints_x) - 1:
        print('Waypoint ' + str(self.current) + ' achieved!')
        self.current = self.current + 1
        self.current_waypoint = [self.waypoints_x[self.current], self.waypoints_y[self.current]]
      else:
        self.finished()
    print('Heading to ' + str(self.current) +'/' + str(len(self.waypoints_x)) + ' (' + str(self.current_waypoint[0]) + ', ' + str(self.current_waypoint[1]) + ') only ' + str(error) + 'm left!')
    self.publish()

  def finished(self):
    rospy.signal_shutdown('Waypoint list finished')

  def publish(self):
    # Convert from NED class to Pose class
    self.nav_sts.header.stamp = rospy.Time.now()
    self.nav_sts.position.north = self.current_waypoint[0]
    self.nav_sts.position.east = self.current_waypoint[1]
    self.nav_sts.position.depth = 10.0
    self.nav_sts.orientation.roll = 0.0
    self.nav_sts.orientation.pitch = 0.0
    self.nav_sts.orientation.yaw = 0.0
    self.waypoint_pub.publish(self.nav_sts)

if __name__ == '__main__':
  try:
    WaypointPublisher()
  except rospy.ROSInterruptException:
    pass