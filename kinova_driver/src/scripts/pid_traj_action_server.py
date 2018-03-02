#! /usr/bin/env python

"""A action server to accept trajectory goals from trajopt
Author: Stephen Hansen
"""

import rospy

import actionlib
import ros_utils

import pid_controller
import actionlib_tutorials.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import kinova_msgs


class JointTrajectoryAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer('/j2s7s300/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._feedback_sub = rospy.Subscriber('/j2s7s300_driver/trajectory_controller/state', control_msgs.msg.FollowJointTrajectoryFeedback, self.feedback_cb)
        self._controller = pid_controller.PIDController()
        self._vel_pub = rospy.Publisher('/j2s7s300_driver/in/joint_velocity',
                                       kinova_msgs.msg.JointVelocity,
                                       queue_size=1)
        self._as.start()


    def feedback_cb(self, feedback):
        """ Cache feedback containing current velocity and position data from the driver 
        to be passed to the movegroup as status information about trajectory execution,

        feedback: a :control_msgs.msg.FollowJointTrajectoryActionFeedback: message
        """
        rospy.loginfo_throttle(10, "feedback updated")
        self._feedback = feedback

    def execute_cb(self, goal):
        """ Main callback for the trajectory action
        Execute the trajectory, stop if premted, report success when done
        goal: a :control_msgs.msg.FollowJointTrajectoryActionGoal: message containing the trajectory and tolerance info
        """
        #TODO use the tolerance variables available:
        # g = control_msgs.msg.FollowJointTrajectoryGoal()
        # g.trajectory
        # g.goal_time_tolerance
        # g.goal_tolerance
        # g.path_tolerance
    
        # The trajectory execution is true unless set false
        success = True
                
        # publish info to the console for the user
        rospy.loginfo('{}: Recieve joint trajectory request'.format(self._action_name))
        
        # start executing the action
        self._controller.load_trajectory(goal.trajectory)

        r = rospy.Rate(100)
        while not rospy.is_shutdown() and not (self._controller.reached_goal and
                                               self._controller.reached_start):
            rospy.loginfo_throttle(5,"reached goal {}, reached_start {}".format(self._controller.reached_goal, self._controller.reached_start))
           
            if self._controller.is_shutdown:
                rospy.loginfo("{}: Controller shut itself down, setting success to false and premted".format(self._action_name))
                success = False
                self._as.set_preempted()
                break

            rospy.loginfo_throttle(5,"publishing command: {}".format(self._controller.cmd))
            self._vel_pub.publish(ros_utils.cmd_to_JointVelocityMsg(self._controller.cmd))

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #TODO there are other reasons to shutdown
                self._controller.shutdown_controller()
                rospy.loginfo('Recieved preemt request: %s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            
            # publish the feedback #TODO is it necissary?
            # feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
            # feedback.feedback = self._feedback
            # feedback.status = feedback.status.ACTIVE
            # self._as.publish_feedback(feedback)
            r.sleep()

        if success:
            #shutdown the controller and report success
            self._controller.shutdown_controller()
            self._result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._controller.is_shutdown = False
            self._as.set_succeeded(self._result)
        
        self._controller.is_shutdown = False
        rospy.loginfo("Action server goal request complete")

if __name__ == '__main__':
    rospy.init_node('follow_joint_trajectory')
    server = JointTrajectoryAction(rospy.get_name())
    rospy.spin()
