#!/usr/bin/env python

# Joystick teleoperation module for the Kinova Jaco arm.

import sys
import rospy
import actionlib
from sensor_msgs.msg import Joy

from jaco_msgs.msg import JoystickCommand, SetFingersPositionAction, SetFingersPositionGoal
from jaco_msgs.srv import Start, Stop;
from jaco_teleop.joystick_interface import JoystickInterface

from time import sleep
import yaml

class JacoTeleopJoy(object):
    """
    Joystick/gamepad teleoperation interface that utilizes the same command that are sent from the dedicated Kinova
    controller.
    """

    def __init__(self):
        """
        Read joystick config from the YAML config file. Subscribe to the joy, set up publishers and action servers.
        Required ROS param: ~config_file (string): Path to the YAML config to load.
        """
        configStream = open(rospy.get_param('~config_file'), 'r')
        # This is a list of all config keys we want to pass to the joystick abstraction layer.
        joystick_config = yaml.load(configStream)

        self.joystick_interface = JoystickInterface(joystick_config)

        rospy.logwarn("Using joystick interface with the following description:\n" + 
                self.joystick_interface.get_description())

        # Topic that executes all received commands with the real arm.
        self.joystick_topic = "/jaco_arm_driver/in/joystick_command"
        self.joystick_pub = rospy.Publisher(self.joystick_topic, JoystickCommand, queue_size=10)

        # Emergency stop service.
        self.emergency_stop_topic = "/jaco_arm_driver/in/stop"
        rospy.wait_for_service(self.emergency_stop_topic)
        self.emergency_stop = rospy.ServiceProxy(self.emergency_stop_topic, Stop)

        # Emergency stop release service.
        self.emergency_start_topic = "/jaco_arm_driver/in/start"
        rospy.wait_for_service(self.emergency_start_topic)
        self.emergency_start = rospy.ServiceProxy(self.emergency_start_topic, Start)

        # Action server for controlling the fingers.
        self.fingersActionTopic = "/jaco_arm_driver/fingers/finger_positions"
        self.fingersClient = actionlib.SimpleActionClient(self.fingersActionTopic,
                                                          SetFingersPositionAction)
        self.fingersClient.wait_for_server()

        # The joystick topic that "commands" this node.
        rospy.Subscriber('/joy', Joy, self.joyCallBack)

    def joyCallBack(self, joy):
        """
        Joystick state change callback. Do all the command translation and command the real arm.
        @param joy: The new joystick state.
        @type joy: sensor_msgs.msg.Joy
        """

        # First of all, check if the emergency stop is not required, and if it is, call the appropriate service.
        if self.joystick_interface.get_value(joy, 'emergency_stop') == 1:
            self.emergency_stop()
            rospy.loginfo("Arm stopped.")
            return

        # Similarly, perform the emergency stop release if needed.
        if self.joystick_interface.get_value(joy, 'emergency_release'):
            self.emergency_start()
            rospy.loginfo("Arm started.")
            return

        # The command that's gonna be sent to the arm.
        command = JoystickCommand()

        # Translate the commands to the arm.
        command.InclineLeftRight = self.joystick_interface.get_value(joy, 'translate_left_right')
        command.InclineForwardBackward = self.joystick_interface.get_value(joy, 'translate_forward_backward')
        command.Rotate = self.joystick_interface.get_value(joy, 'translate_up_down')

        command.MoveLeftRight = self.joystick_interface.get_value(joy, 'rotate_x')
        command.MoveForwardBackward = self.joystick_interface.get_value(joy, 'rotate_y')
        command.PushPull = self.joystick_interface.get_value(joy, 'rotate_wrist')

        if self.joystick_interface.get_value(joy, 'home_arm') == 1:
            command.ButtonValue[2] = 1

        # Send the built command to the real arm.
        self.joystick_pub.publish(command)

        # Fingers are controlled using the action server. The action server only accepts target finger positions,
        # so we only need to set either 0 (min) or 65 (max) target values and stop execution when the control is
        # released.
        fingersTargetPosition = None
        fingersJoyValue = self.joystick_interface.get_value(joy, 'fingers_open_close')
        if fingersJoyValue < -0.1:
            fingersTargetPosition = 65
        elif fingersJoyValue > 0.1:
            fingersTargetPosition = 0

        if fingersTargetPosition is not None:
            # Send the request to the real arm.
            goal = SetFingersPositionGoal()
            goal.fingers.finger1 = float(fingersTargetPosition)
            goal.fingers.finger2 = float(fingersTargetPosition)
            goal.fingers.finger3 = float(fingersTargetPosition)

            self.fingersClient.send_goal(goal)
        else:
            # The control has been released, so immediately stop executing the action.
            self.fingersClient.cancel_all_goals()

def main():
    """Create a ROS node and instantiate the JacoTeleopJoy class."""
    try:
        rospy.init_node('jaco_teleop_joy')
        jtj = JacoTeleopJoy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
