#!/usr/bin/env python
# # @file
# Joystick teleoperation module for the Kinova Jaco arm
#
import roslib

roslib.load_manifest('jaco_teleop')
import rospy
import actionlib
from sensor_msgs.msg import Joy

from jaco_msgs.msg import JoystickCommand, SetFingersPositionAction, SetFingersPositionGoal
from jaco_msgs.srv import Start, Stop;

from time import sleep

## Joystick teleoperation class
class JacoTeleopJoy(object):
    '''Joystick teleoperation interface.
	'''

    ## Instantiate a JacoTeleopJoy object
    def __init__(self):
        self.joystick_topic = "/jaco_arm_driver/in/joystick_command"
        self.joystick_pub = rospy.Publisher(self.joystick_topic, JoystickCommand, queue_size=10)

        self.emergency_stop_topic = "/jaco_arm_driver/in/stop"
        rospy.wait_for_service(self.emergency_stop_topic)
        self.emergency_stop = rospy.ServiceProxy(self.emergency_stop_topic, Stop)

        self.emergency_start_topic = "/jaco_arm_driver/in/start"
        rospy.wait_for_service(self.emergency_start_topic)
        self.emergency_start = rospy.ServiceProxy(self.emergency_start_topic, Start)

        self.fingersActionTopic = "/jaco_arm_driver/fingers/finger_positions"
        self.fingersClient = actionlib.SimpleActionClient(self.fingersActionTopic,
                                                          SetFingersPositionAction)
        self.fingersClient.wait_for_server()

        rospy.Subscriber('/joy', Joy, self.joyCallBack)

        # hack
        self.joy_axis2_moved = False
        self.joy_axis5_moved = False

    ## Listen to the flippers state.
    def joyCallBack(self, joy):
        '''Listen to the joystick state.'''

        if joy.buttons[1] == 1:
            self.emergency_stop()
            rospy.loginfo("Arm stopped.")
            return

        if joy.buttons[3] == 1:
            self.emergency_start()
            rospy.loginfo("Arm started.")
            return

        if not self.joy_axis2_moved and joy.axes[2] != 0:
            self.joy_axis2_moved = True

        if not self.joy_axis5_moved and joy.axes[5] != 0:
            self.joy_axis5_moved = True

        command = JoystickCommand()

        command.InclineLeftRight = joy.axes[0]
        command.InclineForwardBackward = -joy.axes[1]
        if self.joy_axis2_moved:
            command.Rotate = (joy.axes[2] - 1) / 2
        if joy.buttons[4] == 1:
            command.Rotate = 1
        command.MoveLeftRight = -joy.axes[3]
        command.MoveForwardBackward = joy.axes[4]
        if self.joy_axis5_moved:
            command.PushPull = (joy.axes[5] - 1) / 2
        if joy.buttons[5] == 1:
            command.PushPull = 1

        if joy.buttons[7] == 1:
            command.ButtonValue[2] = 1

        self.joystick_pub.publish(command)

        fingersTargetPosition = None
        if joy.axes[7] < -0.1 or joy.buttons[0] == 1:
            fingersTargetPosition = 65
        if joy.axes[7] > 0.1 or joy.buttons[2] == 1:
            fingersTargetPosition = 0

        if fingersTargetPosition is not None:
            goal = SetFingersPositionGoal()
            goal.fingers.finger1 = float(fingersTargetPosition)
            goal.fingers.finger2 = float(fingersTargetPosition)
            goal.fingers.finger3 = float(fingersTargetPosition)

            self.fingersClient.send_goal(goal)
        else:
            self.fingersClient.cancel_all_goals()

## Create a ROS node and instantiate the JacoTeleopJoy class.
def main():
    '''Create a ROS node and instantiate the JacoTeleopJoy class.'''
    try:
        rospy.init_node('jaco_teleop_joy')
        jtj = JacoTeleopJoy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()