#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('control')
from cat_arch.msg import Setpoints
from sensor_msgs.msg import Joy
from math import *
import numpy as np

class Controller:
    def __init__(self):

        # Publisher
        self.pub_thrusters_setpoints = rospy.Publisher('/setpoints',
                                             Setpoints,
                                             queue_size = 2)
        # Subscriber
        rospy.Subscriber("/control/map_ack_data",
                         Joy,
                         self.joystick_data_callback,
                         queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        # Message
        self.msg = Setpoints()
        self.msg.setpoints = np.array([0.0, 0.0])
       

    def joystick_data_callback(self, data):
        rospy.loginfo( data.axes)

        forward = data.axes[6]
        clockwise_yaw = data.axes[11]
        aux = np.array([0.0, 0.0])
        aux[0] = forward + clockwise_yaw
        aux[1] = forward - clockwise_yaw
        self.msg.setpoints = aux.clip(min=-1.0, max=1.0)
        

    def timer_callback(self, event):
        """ Callback of the timer """
        self.pub_thrusters_setpoints.publish(self.msg)


if __name__ == '__main__':

    rospy.init_node('Controller')
    Controller()
    rospy.spin()


   