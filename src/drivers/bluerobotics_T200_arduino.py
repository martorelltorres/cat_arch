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
from xiroi.msg import Setpoints, Current
from std_srvs.srv import Empty, EmptyResponse
from m4atx_battery_monitor.msg import PowerReading

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino_node', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        self.port = rospy.get_param("~port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A6008iB3-if00-port0")
        self.baud = int(rospy.get_param("~baud",57600))
        self.timeout = rospy.get_param("~timeout",0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        self.left_thruster_pin = rospy.get_param("~left_thruster_pin", 0)
        self.right_thruster_pin = rospy.get_param("~right_thruster_pin", 1)
        self.current_sensor_pin = rospy.get_param("~current_sensor_pin", 0)
        self.position_lights_pin = rospy.get_param("~position_ligts_pin", 11)
        self.strobe_light_pin = rospy.get_param("~strobe_light_pin", 10)
        self.enable_lights_pin = rospy.get_param("enable_lights_pin",9)
	self.current_sensor_power_pin = rospy.get_param("~current_sensor_power_pin", 3)
        self.first_time=True;

        #Slew rate parameters
        self.rising_rate = 0.3 #0.3
        self.falling_rate = 1.0 #1
        self.setpoint_limit = 0.6 #1

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Callback for thrusters
        rospy.Subscriber("setpoints", Setpoints, self.callback, queue_size = 1)
        self.setpoints_corrected = rospy.Publisher("setpoints_corrected", Setpoints, queue_size=1)
        self.thruster_current = rospy.Publisher("thruster_current",Current, queue_size=1)

        # Callback for BatteryInfo
        rospy.Subscriber("/sensors/battery", PowerReading, self.battery_info)
	self.soc = 0

        #Services
        self.thrusters_enabled = False
        # Create services
        self.e_srv = rospy.Service('control/enable_thrusters', Empty, self.enable_thrusters_srv)
        self.d_srv = rospy.Service('control/disable_thrusters', Empty, self.disable_thrusters_srv)
        self.e_pl_srv = rospy.Service('enable_position_lights', Empty, self.enable_lights_srv)
        self.d_pl_srv = rospy.Service('disable_position_lights', Empty, self.disable_lights_srv)
	self.e_sl = rospy.Service('enable_strobe_light', Empty, self.enable_strobe_light_srv)
     	self.d_sl = rospy.Service('disable_strobe_light', Empty, self.disable_strobe_light_srv)

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        # Make the connection
        self.controller.connect()
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        self.controller.pin_mode(self.current_sensor_power_pin, 1)
        self.controller.pin_mode(self.position_lights_pin, 1)
        self.controller.pin_mode(self.strobe_light_pin, 1)
	self.controller.pin_mode(self.enable_lights_pin,1)
        self.controller.digital_write(self.current_sensor_power_pin,1)
        self.controller.digital_write(self.position_lights_pin,0)
        self.controller.digital_write(self.strobe_light_pin,0)
	self.controller.digital_write(self.enable_lights_pin,1)
        while not rospy.is_shutdown():
            current = self.controller.analog_read(self.current_sensor_pin)
            # publish
            msg = Current()
            msg.header.stamp = rospy.Time.now()
            msg.current = ((current/1024.0*5.0) - 2.41)/0.066
            self.thruster_current.publish(msg)
	    self.strobe_led()
            r.sleep()

    def disable_thrusters_srv(self,req):
        """ Disable thruster service """
        self.controller.servo_write(self.left_thruster_pin,1500)
        self.controller.servo_write(self.right_thruster_pin,1500)
        self.thrusters_enabled = False
        rospy.loginfo("THRUSTERS DISABLED")
        return EmptyResponse()
    

    def enable_thrusters_srv(self,req):
        """ Enable thruster service """
        self.thrusters_enabled = True
        rospy.loginfo("THRUSTERS ENABLED")
        return EmptyResponse()

    def enable_lights_srv(self, req):
        self.controller.digital_write(self.position_lights_pin,1)
        return EmptyResponse()
	
    def disable_lights_srv(self, req):
        self.controller.digital_write(self.position_lights_pin,0)
        return EmptyResponse()

    def enable_strobe_light_srv(self, req):
	self.controller.digital_write(self.strobe_light_pin,1)
	return EmptyResponse()

    def disable_strobe_light_srv(self, req):
	self.controller.digital_write(self.strobe_light_pin,0)
	return EmptyResponse()
	
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
        actual_time=rospy.Time.now()
        message_time=msg.header.stamp
        delay_threshold=rospy.Duration(0,650000)
        time_dif=actual_time-message_time

        # if time_dif<delay_threshold:
        msg0=msg.setpoints[0];
        msg1=msg.setpoints[1];
        time=rospy.Time.now().to_sec()

        if(self.first_time):
            self.controller.servo_write(self.left_thruster_pin,1500+msg0*400)
            self.controller.servo_write(self.right_thruster_pin,1500+msg1*400)
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

        if self.thrusters_enabled:
            #print str(1500+msg0*400) + ' ' + str(1500+msg1*400)
            self.controller.servo_write(self.left_thruster_pin,1500+msg0*400)
            self.controller.servo_write(self.right_thruster_pin,1500+msg1*400)

        self.old_time = time
        self.old_msg0 = msg0
        self.old_msg1 = msg1
        msg.setpoints = [msg0, msg1]
        self.setpoints_corrected.publish(msg)
    
    def battery_info(self, data):
        self.soc = data.input_soc
    
    def strobe_led(self):
        #print(self.soc)
        if 100.0 >= self.soc and self.soc >= 75.0:
            time.sleep(2.5)
            for i in range(4):
                self.controller.digital_write(self.strobe_light_pin, 1)
                time.sleep(0.2)
                self.controller.digital_write(self.strobe_light_pin, 0)
                time.sleep(0.2)

        elif 75.0 > self.soc and self.soc >= 50.0:
            time.sleep(2.9)
            for i in range(3):
                self.controller.digital_write(self.strobe_light_pin, 1)
		time.sleep(0.2)
                self.controller.digital_write(self.strobe_light_pin, 0)
                time.sleep(0.2)
                
        elif 50.0 > self.soc and self.soc >= 25.0:
            time.sleep(3.3)
            for i in range(2):
                self.controller.digital_write(self.strobe_light_pin, 1)
		time.sleep(0.2)
                self.controller.digital_write(self.strobe_light_pin, 0)
		time.sleep(0.2)

        elif 25.0 > self.soc and self.soc >= 0.0:
            time.sleep(3.7)
            for i in range(1):
                self.controller.digital_write(self.strobe_light_pin, 1)
		time.sleep(0.2)
                self.controller.digital_write(self.strobe_light_pin, 0)
		time.sleep(0.2)

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
