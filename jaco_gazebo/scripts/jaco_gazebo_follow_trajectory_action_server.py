#! /usr/bin/env python

import roslib
import rospy
import actionlib
from control_msgs.msg import *
import math
import getopt

class JacoFollowJointTrajectoryAction(object):
  # create messages that are used to publish feedback/result
  _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
  _result   = control_msgs.msg.FollowJointTrajectoryResult()

  def __init__(self, name,robotName):
    self._action_name = name
    self._ang_thres = 0.02
    self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb)
    rospy.Subscriber(robotName + "_arm/" + robotName + "_joint1_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_joint_1_callback)
    rospy.Subscriber(robotName + "_arm/" + robotName + "_joint2_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_joint_2_callback)
    rospy.Subscriber(robotName + "_arm/" + robotName + "_joint3_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_joint_3_callback)
    rospy.Subscriber(robotName + "_arm/" + robotName + "_joint4_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_joint_4_callback)
    rospy.Subscriber(robotName + "_arm/" + robotName + "_joint5_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_joint_5_callback)
    rospy.Subscriber(robotName + "_arm/" + robotName + "_joint6_position_controller/state", control_msgs.msg.JointControllerState, self.subscriber_joint_6_callback)

    self._joint_1_pub = rospy.Publisher(robotName + "_arm/" + robotName + "_joint1_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self._joint_2_pub = rospy.Publisher(robotName + "_arm/" + robotName + "_joint2_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self._joint_3_pub = rospy.Publisher(robotName + "_arm/" + robotName + "_joint3_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self._joint_4_pub = rospy.Publisher(robotName + "_arm/" + robotName + "_joint4_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self._joint_5_pub = rospy.Publisher(robotName + "_arm/" + robotName + "_joint5_position_controller/command", std_msgs.msg.Float64, queue_size=3)
    self._joint_6_pub = rospy.Publisher(robotName + "_arm/" + robotName + "_joint6_position_controller/command", std_msgs.msg.Float64, queue_size=3)

    self.finished_trajectory = False
    self._as.start()
  def execute_cb(self, goal):

    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    #self._joint_1_pub.publish(60.0*180.0/math.pi)
    #self._joint_2_pub.publish(60.0*180.0/math.pi)
    #print 'Sending goal ' + str(goal)
    #print 'Number of points received ' + str(len(goal.trajectory.points))
    r = rospy.Rate(7) # 10hz
    for currPointInd in range(len(goal.trajectory.points)):
        currentPoint = goal.trajectory.points[currPointInd]
        self._joint_1_pub.publish(currentPoint.positions[0])
        self._joint_2_pub.publish(currentPoint.positions[1])
        self._joint_3_pub.publish(currentPoint.positions[2])
        self._joint_4_pub.publish(currentPoint.positions[3])
        self._joint_5_pub.publish(currentPoint.positions[4])
        self._joint_6_pub.publish(currentPoint.positions[5])
	r.sleep()
	if currPointInd == len(goal.trajectory.points) -1:
	    r.sleep()
	    if math.fabs(currentPoint.positions[0] - self._position_joint_1) < self._ang_thres and math.fabs(currentPoint.positions[1] - self._position_joint_2) < self._ang_thres and (currentPoint.positions[2] - self._position_joint_3) < self._ang_thres and (currentPoint.positions[3] - self._position_joint_4) < self._ang_thres and (currentPoint.positions[4] - self._position_joint_5) < self._ang_thres and (currentPoint.positions[5] - self._position_joint_6) < self._ang_thres:
		self._result.error_code = 0
	    else:
		self._result.error_code = -5
    rospy.loginfo('%s: Succeeded' % self._action_name)
    print 'error_code: ' + str(self._result.error_code)
    self._as.set_succeeded(self._result)

  def subscriber_joint_1_callback(self, data):
	self._position_joint_1 = data.process_value
	#print self._position_joint_1

  def subscriber_joint_2_callback(self, data):
	self._position_joint_2 = data.process_value
	#print self._position_joint_2

  def subscriber_joint_3_callback(self, data):
	self._position_joint_3 = data.process_value
	#print self._position_joint_3

  def subscriber_joint_4_callback(self, data):
	self._position_joint_4 = data.process_value
	#print self._position_joint_4

  def subscriber_joint_5_callback(self, data):
	self._position_joint_5 = data.process_value
	#print self._position_joint_5

  def subscriber_joint_6_callback(self, data):
	self._position_joint_6 = data.process_value
	#print self._position_joint_6

if __name__ == '__main__':
  try:
    opts, args = getopt.getopt(sys.argv[1:],"")
    if len(args)!=1:
      print 'jaco_gazebo_follow_trajectory_action_server.py <mico/jaco>'
  except getopt.GetoptError:
      print 'jaco_gazebo_follow_trajectory_action_server.py <mico/jaco>'
      sys.exit(2)
  rospy.init_node(args[0] + '_joint_trajectory_action_server')
  JacoFollowJointTrajectoryAction(rospy.get_name(),args[0])
  rospy.spin()

