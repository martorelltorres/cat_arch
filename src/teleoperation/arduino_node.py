#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
""" 

import rospy
from ros_arduino_python.arduino_driver import Arduino
import os, time
import thread
from serial.serialutil import SerialException
from xiroi.msg import Setpoints

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino_node', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud",57600))
        self.timeout = rospy.get_param("~timeout",0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        self.left_thruster_pin = rospy.get_param("~left_thruster_pin", 0)
        self.right_thruster_pin = rospy.get_param("~right_thruster_pin", 1)
        self.first_time=True;

        #Slew rate parameters
        self.rising_rate = 0.1
        self.falling_rate = 1
        self.setpoint_limit = 0.8
      
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Callback for thrusters
        rospy.Subscriber("/setpoints", Setpoints, self.callback)
        self.setpoints_corrected = rospy.Publisher("/setpoints_corrected", Setpoints, queue_size=1)
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        # Make the connection
        self.controller.connect()
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        while not rospy.is_shutdown():
            r.sleep()
    
    def limitSetpoint(self, msg, time, old_msg, old_time):
        #Rate calculation
        rate = (msg - old_msg)/(time - old_time)
       
        # Case 1
        if (rate>0 and msg>0 and rate > self.rising_rate):
            msg = (time-old_time)*self.rising_rate + old_msg
        # Case 2
        elif (rate<0 and msg>=0 and rate < -self.falling_rate): 
            msg = -(time-old_time)*self.falling_rate + old_msg
        # Case 3
        elif (rate<0 and msg<0  and rate < -self.rising_rate):
            msg = -(time-old_time)*self.rising_rate + old_msg
        # Case 4
        elif (rate>0 and msg<=0 and rate > self.falling_rate):
            msg = (time-old_time)*self.falling_rate + old_msg

        return msg

    # Service callback functions
    def callback(self, msg):
    #     #1900 => max thrust positive
    #     #1500 => 0A
    #     #1100 => max thrust negative
        msg0=msg.setpoints[0];
        msg1=msg.setpoints[1];
        time=msg.header.stamp.to_sec()

        if(self.first_time):
            self.controller.servo_write(self.left_thruster_pin,msg0)
            self.controller.servo_write(self.right_thruster_pin,msg1)
            self.old_time = time
            self.old_msg0 = msg0
            self.old_msg1 = msg1
            self.first_time = False
            return

        msg0 = self.limitSetpoint(msg0, time, self.old_msg0, self.old_time)
        msg1 = self.limitSetpoint(msg1, time, self.old_msg1, self.old_time)

        if msg0 > self.setpoint_limit:
            msg0 = self.setpoint_limit
        elif msg0 < -self.setpoint_limit:
            msg0 = -self.setpoint_limit

        if msg1 > self.setpoint_limit:
            msg1 = self.setpoint_limit
        elif msg1 < -self.setpoint_limit:
            msg1 = -self.setpoint_limit
                

        self.controller.servo_write(self.left_thruster_pin,1500+msg0*400)
        self.controller.servo_write(self.right_thruster_pin,1500+msg1*400)
     
        self.old_time = time
        self.old_msg0 = msg0
        self.old_msg1 = msg1

        msg.setpoints = [msg0, msg1]
        self.setpoints_corrected.publish(msg)
 
    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        
        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            rospy.loginfo("Serial port closed.")
            os._exit(0)

if __name__ == '__main__':


    try:
        ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)