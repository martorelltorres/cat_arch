#!/usr/bin/env python

"""@@Recovery actions node is used to handle requests for recovery actions coming
from any node@@"""

"""
Created on Mar 25 2013
@author: narcis palomeras


"""
import rospy
from std_srvs.srv import Empty, EmptyRequest
from xiroi.srv import RecoveryAction, RecoveryActionRequest, RecoveryActionResponse
from xiroi.msg import Setpoints
from ned_tools import utils_ros

class RecoveryActions(object):
  """ This class is able to handle recovery requests coming from all the
    nodes """

  def __init__(self):
    """ Init the class """
    self.name = 'RecoveryActions'
    
    # Create publisher
    self.pub_thrusters = rospy.Publisher("control/thrusters_data",
                                         Setpoints,
                                         queue_size = 2)

    # Init service clients
    rospy.loginfo("[%s]: waiting for services", self.name)
    try:
      #Server allocated in operator.py
      rospy.wait_for_service('control/abort_mission', 20)
      self.abort_mission_srv = rospy.ServiceProxy(
                            'control/abort_mission', Empty)
    except rospy.exceptions.ROSException:
      rospy.logfatal('abort_mission service FAILED')

    try:
      #Server allocated in arduino_node.py
      rospy.wait_for_service('control/disable_thrusters', 20)
      self.disable_thrusters_srv = rospy.ServiceProxy(
                            'control/disable_thrusters', Empty)
    except rospy.exceptions.ROSException:
      rospy.logfatal('disable_thrusters service FAILED')

    # Create service
    self.recovery_srv = rospy.Service('safety/recovery_action',
                                    RecoveryAction,
                                    self.recovery_action_srv)
    # Show message
    rospy.loginfo("[%s]: initialized", self.name)

  def recovery_action_srv(self, req):
    """ Callback of recovery action service """
    rospy.loginfo('[%s]: received recovery action', self.name)
    self.recovery_action(req.errors_level)
    ret = RecoveryActionResponse()
    ret.attempted = True
    return ret

  def recovery_action(self, error):
    """ This method calls the appropiate method to handle the input code """
    if error == RecoveryActionRequest.INFORMATIVE:
      rospy.loginfo("[%s]: recovery action %s: INFORMATIVE",
                    self.name, error)
    elif error == RecoveryActionRequest.ABORT_MISSION:
      rospy.loginfo("[%s]: recovery action %s: ABORT_MISSION",
                    self.name, error)
      self.abort_mission()
    elif error == RecoveryActionRequest.DISABLE_THRUSTERS:
      rospy.loginfo("[%s]: recovery action %s: DISABLE_THRUSTERS",
                    self.name, error)
      self.disable_thrusters()
    else:
      rospy.loginfo("[%s]: recovery action %s: INVALID ERROR CODE",
                    self.name, error)

  def abort_mission(self):
    """ This method handles abort mission """
    rospy.loginfo("[%s]: abort mission", self.name)
    try:
      self.abort_mission_srv(EmptyRequest())
    except rospy.exceptions.ROSException:
      rospy.logerr('[%s]: error aborting the mission', self.name)

  def disable_thrusters(self):
    rospy.loginfo("[%s]: Stopping thrusters", self.name)
    try:
      self.disable_thrusters_srv(EmptyRequest())
    except rospy.exceptions.ROSException:
      rospy.logerr('[%s]: error disabling thrusters', self.name)

  def no_disable_thrusters_message(self, event):
    """ Timer to show an error in disable thrusters service """
    rospy.logfatal('[%s]: error creating client to disable thrusters', self.name)

  def no_captain_clients_message(self, event):
    """ Timer to show an error if unavailable captain service """
    rospy.logfatal('[%s]: error creating some captain clients', self.name)