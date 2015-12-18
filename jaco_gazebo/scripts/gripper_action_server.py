#! /usr/bin/env python

import roslib
import rospy
import actionlib
from control_msgs.msg import *
import math

class MicoGripperAction(object):
  # create messages that are used to publish feedback/result
  _feedback = control_msgs.msg.GripperCommandFeedback()
  _result   = control_msgs.msg.GripperCommandResult()

  def __init__(self, name):
    self._action_name = name
    self._position_finger_1 = 0.0
    self._position_finger_2 = 0.0
    self._previous_position_finger_1 = -1.0
    self._previous_position_finger_2 = -1.0
    self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb)
    rospy.Subscriber("mico_arm/mico_joint_finger_1_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_finger_1_callback)
    rospy.Subscriber("mico_arm/mico_joint_finger_2_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_finger_2_callback)
    self._joint_1_pub = rospy.Publisher("mico_arm/mico_joint_finger_1_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self._joint_2_pub = rospy.Publisher("mico_arm/mico_joint_finger_2_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self.gripper_not_moving = False
    self._as.start()
  def execute_cb(self, goal):

    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    r = rospy.Rate(5) # 10hz
    #self._joint_1_pub.publish(60.0*180.0/math.pi)
    #self._joint_2_pub.publish(60.0*180.0/math.pi)
    #print 'Sending goal ' + str(goal)
    self._joint_1_pub.publish(goal.command.position)
    self._joint_2_pub.publish(goal.command.position)
    r.sleep()
    self.gripper_not_moving = False
    while not self.gripper_not_moving:
	    #print 'Closing gripper'
	    current_finger_1 = self._position_finger_1
	    #print self._position_finger_1
	    current_finger_2 = self._position_finger_2
	    if math.fabs(current_finger_1 - self._previous_position_finger_1) < 0.001 and math.fabs(current_finger_2 - self._previous_position_finger_2) < 0.001:
		#print 'Not moving anymore'
		self.gripper_not_moving = True
	    else:
		#print 'still moving'
		self.gripper_not_moving = False
		self._previous_position_finger_1 = current_finger_1
		self._previous_position_finger_2 = current_finger_2
	    r.sleep()
    if self.gripper_not_moving:
	#print 'Not moving anymore'
	if math.fabs(self._position_finger_1 - goal.command.position) < 0.001 and math.fabs(self._position_finger_2 - goal.command.position) < 0.001:
	    #print 'Sending success'
	    reached_goal_ = True
        else:
	    #print 'Other condition'
	    reached_goal_ = False
    self._result.reached_goal = reached_goal_
    self._result.stalled = False
    self._result.position = self._position_finger_1
    rospy.loginfo('%s: Succeeded' % self._action_name)
    self._as.set_succeeded(self._result)

  def subscriber_finger_1_callback(self, data):
	self._position_finger_1 = data.process_value
	#print self._position_finger_1

  def subscriber_finger_2_callback(self, data):
	self._position_finger_2 = data.process_value
	#print self._position_finger_2

if __name__ == '__main__':
  rospy.init_node('gripper_action_server')
  MicoGripperAction(rospy.get_name())
  rospy.spin()

