#!/usr/bin/env python

# ROS imports
import roslib
roslib.load_manifest('xiroi')
import rospy
from safety_lib import SafetyManager

if __name__ == '__main__':
  try:
    rospy.init_node('safety', log_level=rospy.INFO)
    safety = SafetyManager()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
