#! /usr/bin/env python
"""LogitechFX10 controler node."""

import rospy
from std_srvs.srv import Empty

from joystick_base import JoystickBase

class LogitechFX10(JoystickBase):
    """LogitechFX10 controler class."""

    """
        This class inherent from JoystickBase. It has to overload the
        method update_joy(self, joy) that receives a sensor_msgs/Joy
        message and fill the var self.joy_msg as described in the class
        JoystickBase.
        From this class it is also possible to call services or anything
        else reading the buttons in the update_joy method.
    """

    def __init__(self, name):
        """Class Constructor."""
        JoystickBase.__init__(self, name)

    def update_joy(self, joy):
        """Receive joy raw data."""
        """ Transform FX10 joy data into 12 axis data (pose + twist)
        and sets the buttons that especify if position or velocity
        commands are used in the teleoperation."""

        # JOYSTICK  DEFINITION:
        LEFT_JOY_HORIZONTAL = 0     # LEFT+, RIGHT-
        LEFT_JOY_VERTICAL = 1       # UP+, DOWN-
        LEFT_TRIGGER = 2            # NOT PRESS 1, PRESS -1
        RIGHT_JOY_HORIZONTAL = 3    # LEFT+, RIGHT-
        RIGHT_JOY_VERTICAL = 4      # UP+, DOWN-
        RIGHT_TRIGGER = 5           # NOT PRESS 1, PRESS -1
        CROSS_HORIZONTAL = 6        # LEFT+, RIGHT-
        CROSS_VERTICAL = 7          # UP+, DOWN-
        BUTTON_A = 0
        BUTTON_B = 1
        BUTTON_X = 2
        BUTTON_Y = 3
        BUTTON_LEFT = 4
        BUTTON_RIGHT = 5
        BUTTON_BACK = 6
        BUTTON_START = 7
        BUTTON_LOGITECH = 8
        BUTTON_LEFT_JOY = 9
        BUTTON_RIGHT_JOY = 10
        MOVE_UP = 1
        MOVE_DOWN = -1
        MOVE_LEFT = 1
        MOVE_RIGHT = -1

        self.joy_msg.header = joy.header

        self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = joy.axes[ LEFT_JOY_VERTICAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = -joy.axes[RIGHT_JOY_HORIZONTAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = joy.axes[RIGHT_JOY_VERTICAL]
        self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = -joy.axes[LEFT_JOY_HORIZONTAL]

if __name__ == '__main__':
    """ Initialize the logitech_fx10 node. """
    try:
        rospy.init_node('logitech_fx10')
        map_ack = LogitechFX10(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass