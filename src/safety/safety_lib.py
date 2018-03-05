#!/usr/bin/env python

"""package SafetyManager is used to check:
  - Absolute timeout
  - Navigator timeout
  - Teleoperation timeout
  - Battery voltage


"""

# ROS imports
import roslib
roslib.load_manifest('xiroi')
import rospy
import rosparam

from std_msgs.msg import String
from auv_msgs.msg import NavSts
from diagnostic_msgs.msg import DiagnosticStatus
from xiroi.msg import TotalTime
from m4atx_battery_monitor.msg import PowerReading
from sensor_msgs.msg import NavSatFix

from std_srvs.srv import Empty, EmptyResponse

from ned_tools import utils_ros
from recovery_actions_lib import RecoveryActions
from diagnostic_helper import DiagnosticHelper


class Monitor(object):
  def __init__(self, name):
    self.name = name
    self.diagnostic = 0

  def setDiagnostics(self, diagnostic):
    self.diagnostic = diagnostic

class ValueMonitor(Monitor):
  """Class that monitors a value and if it grows over max value,
  calls on_max function. With minimum values calls on_min. Default
  arguments take care of only-max or only-min monitors.
  The user MUST call update() function with the value to monitor
  to pass the value to the ValueMonitor."""

  def __init__(self, name, min_value = None, max_value = None, callback_period = 0.1, on_min = None, on_max = None, show_warn = True):
    """Initialize the class variables with the input arguments
    :param name: Name of the ValueMonitor
    :param min_value: If monitored value is less than min_value, on_min gets called
    :param max_value: If monitored value is greater than max_value, on_max gets called
    :param callback_period: Timer period that will check min and max bounds
    :param on_min: Function called when the monitored value is less than min_value
    :param on_max: Function called when the monitored value is greater than max_value
    :param show_warn: Print rospy warning messages
    """
    Monitor.__init__(self, name)
    self.min = min_value
    self.max = max_value
    self.on_min = on_min
    self.on_max = on_max
    self.value = None
    self.show_warn = show_warn  # In case some function does not need to warn all time
    rospy.Timer(rospy.Duration(callback_period), self.__callback)

  def __callback(self, event):
    """Timer callback called at declared period
    :param event: Timer event (see rospy Timer documentation)
    """
    if self.value is not None:
      if self.max is not None:
        if self.value > self.max and self.on_max is not None:
          if self.show_warn:
            rospy.logwarn("[%s]: value exceeded its maximum: %s", self.name, self.max)
          self.diagnostic.add(self.name + " value exceeded its maximum", str(self.value))
          self.diagnostic.setLevel(DiagnosticStatus.WARN ,'WARN')
          self.on_max(self.name)
      if self.min is not None:
        if self.value < self.min and self.on_min is not None:
          if self.show_warn:
            rospy.logwarn("[%s]: value exceeded its minimum: %s", self.name, self.min)
          self.diagnostic.add(self.name + " value exceeded its minimum", str(self.value))
          self.diagnostic.setLevel(DiagnosticStatus.WARN ,'WARN')
          self.on_min(self.name)

  def update(self, value):
    """User callable function to update the value to monitor
    :param value: New value to update the monitor stored value
    """
    self.value = value
    self.diagnostic.add(self.name + " Value update", str(value))
    self.diagnostic.setLevel(DiagnosticStatus.OK ,'OK')

class TimeoutMonitor(Monitor):
  """Class that monitors a freq and if it less than a minimum freq
  value calls on_timeout.
  The user MUST call update() function to update the last call
  time value."""

  def __init__(self, name, timeout, callback_period = 0.1, on_timeout = None, wait_time = 30.0):
    """Initialize the class variables with the input arguments
    :param name: Name of the TimeoutMonitor
    :param timeout: Value in seconds of the desired timeout
    :param callback_period: Timer period that will check the timeout
    :param on_timeout: Function called when timeout has passed
    :param wait_time: Initialization time, before checking timeout
    """
    Monitor.__init__(self, name)
    rospy.Timer(rospy.Duration(callback_period), self.__callback)
    self.wait_time = wait_time
    self.timeout = timeout
    self.last_time = rospy.Time.now()
    self.on_timeout = on_timeout
    self.initialization_time = self.last_time

  def __callback(self, event):
    """Timer callback called at declared period
    :param event: Timer event (see rospy Timer documentation)
    """
    if (rospy.Time.now() - self.initialization_time).to_sec() > self.wait_time:
      dt = (rospy.Time.now() - self.last_time).to_sec()
      if dt > self.timeout:
        self.diagnostic.add(self.name + " timeout", str(dt))
        self.diagnostic.setLevel(DiagnosticStatus.WARN ,'WARN')
        self.on_timeout(self.name)

  def update(self):
    """User callable function to update the last time called"""
    self.last_time = rospy.Time.now()
    self.diagnostic.add(self.name + " Time update", str(self.last_time))
    self.diagnostic.setLevel(DiagnosticStatus.OK ,'OK')


class SafetyManager(object):
  """ Safety class that setups all the value monitors and
  frequency monitors to check the correct operation of the
  robot
  """
  def __init__(self):
    """ Constructor function """
    # Init default variables
    self.init_time = rospy.Time.now()
    self.published_config = False
    # self.emerge = False
    self.absolute_timeout = 20      # Seconds
    self.communication_timeout = 15 # Seconds
    self.min_cell_voltage = 11.95   # Volts
    self.recoverer = RecoveryActions()
    self.name = 'SafetyManager'

    # Set up diagnostics
    self.diagnostic = DiagnosticHelper(self.name, "hw")

    # Set Safety Monitors
    # self.global_timeout_monitor = TimeoutMonitor('GlobalTimerMonitor',
    #                                               timeout = self.absolute_timeout,
    #                                               on_timeout = self.abort_mission)

    # self.navigation_timeout_monitor = TimeoutMonitor('NavigationTimeoutMonitor',
    #                                               timeout = 120.0,
    #                                               on_timeout = self.abort_mission,
    #                                               wait_time = 60.0)
    self.communication_timeout_monitor = TimeoutMonitor('JoyTimeoutMonitor',
                                                timeout = self.communication_timeout,
                                                on_timeout = self.recoverer.disable_thrusters,
                                                wait_time = 120.0)
    self.gps_timeout_monitor = TimeoutMonitor('GpsTimeoutMonitor',
                                                timeout = self.communication_timeout,
                                                on_timeout = self.recoverer.disable_thrusters,
                                                wait_time = 120.0)
    self.minimum_cell_voltage_monitor  = ValueMonitor('MinCellVoltage',
                                                      min_value = self.min_cell_voltage,
                                                      on_min = self.recoverer.disable_thrusters)
    # self.global_timeout_monitor.setDiagnostics(self.diagnostic)
    # self.navigation_timeout_monitor.setDiagnostics(self.diagnostic)
    self.communication_timeout_monitor.setDiagnostics(self.diagnostic)
    self.gps_timeout_monitor.setDiagnostics(self.diagnostic)
    self.minimum_cell_voltage_monitor.setDiagnostics(self.diagnostic)

    # Setup Subscribers
    # rospy.Subscriber("/navigation/nav_sts",NavSts,self.nav_sts_callback,queue_size = 1)
    # rospy.Subscriber("/navigation/nav_sts_acoustic",NavSts,self.nav_sts_callback,queue_size = 1)
    rospy.Subscriber("sensors/battery", PowerReading, self.m4atx_callback, queue_size = 1)
    rospy.Subscriber("control/ack_ack", String, self.ack_ack_callback,queue_size = 1)
    rospy.Subscriber("sensors/gps_raw", NavSatFix, self.pose_callback,queue_size = 1)

    # Setup TotalTime Publisher
    self.tt_pub = rospy.Publisher(
         "safety/total_time",
         TotalTime,
         queue_size = 2)

    # Create reset timeout service
    self.reset_timeout_srv = rospy.Service('safety/reset_timeout',
                                           Empty,
                                           self.reset_timeout)

    # Timer to check absolute timeout. Period must be 0.1 sec
    rospy.Timer(rospy.Duration(0.1), self.check_absolute_timeout)

    self.diagnostic.add("Safety", "Ready!")
    self.diagnostic.setLevel(DiagnosticStatus.OK ,'OK')

  # # Subscriber callbacks
  # def nav_sts_callback(self, nav):
  #   """ NavSts callback to:
  #    * Update the navigation timeout
  #    * Check minimum altitude
  #    * Check maximum depth
  #    * Check zero velocity depth
  #   :param nav: Navigation message (NavSts)
  #   """
  #   self.navigation_timeout_monitor.update()


  def m4atx_callback(self, m4atx):
    """ Battery Management System callback to update the
    voltage monitor
    :param bms: BMS message
    """
    self.minimum_cell_voltage_monitor.update(m4atx.volts_read_value[0])

  def ack_ack_callback(self, event):
    """ Map Acknowledge Acknoledgement callback to update the
    teleoperation monitor
    :param event: Keyboard event
    """
    self.communication_timeout_monitor.update()

  def pose_callback(self, event):
    """ Check if we are getting USBL->Modem messages """
    self.gps_timeout_monitor.update()

  # Monitor callbacks
  def disable_thrusters(self, monitor_name):
    """ Stop and emerge routine to call the corresponding recoverer function
    :param monitor_name: Name of the monitor to show in the messages
    """
    rospy.logwarn("[%s]: %s timeout reached! THRUSTERS OFF", self.name, monitor_name)
    self.recoverer.disable_thrusters()

  def check_absolute_timeout(self, event):
    """ Check absolute timeout of the robot
    :param event: Timer event
    """
    self.elapsed_time = (rospy.Time.now() - self.init_time).to_sec()
    # self.global_timeout_monitor.update()
    # Publish total time
    self.publish_total_time()
    # Publish config once
    if self.elapsed_time > 30.0 and not self.published_config:
      self.publish_config()

  def publish_total_time(self):
    """ Publish a TotalTime message with the elapsed time """
    msg = TotalTime()
    msg.total_time = int(self.elapsed_time)
    msg.timeout = int(self.absolute_timeout)
    self.tt_pub.publish(msg)

  def publish_config(self):
    """ Publish the rosparam server config through a topic """
    self.published_config = True
    pub = rospy.Publisher("safety/configuration",
                          String,
                          queue_size = 2)

    msg = String()
    parameters_list = rosparam.list_params('/')
    data = '\n'
    for parameter in parameters_list:
        data = data + 'Parameter: ' + parameter + '\n'
        data = data + 'Value: ' + str(rospy.get_param(parameter)) + '\n\n'
    msg.data = data
    pub.publish(msg)

  def reset_timeout(self, req):
    """ Service used to reset timeout
    :param req: Timeout empty requests
    """
    self.absolute_timeout = self.absolute_timeout + (rospy.Time.now() - self.init_time).to_sec()
    return EmptyResponse()

