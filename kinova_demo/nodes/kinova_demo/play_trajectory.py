#!/usr/bin/python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import csv
import numpy as np
import rospy
from robot_control_modules import argumentParser, joint_position_client
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEBUG = False  # { True, False }
PATH = '/home/henrique/Documents/kinova/trajectories'

if __name__ == '__main__':
    try:
        # Parse command line arguments

        prefix, nbJoints = argumentParser(None)

        # Read motion

        t = None
        q_mat = None
        qdot_mat = None
        tau_mat = None

        with open('{path}/time.csv'.format(path=PATH)) as f:
            reader = csv.reader(f)
            t = [float(i) for i in list(reader)[0]]

        with open('{path}/positions.csv'.format(path=PATH)) as f:
            reader = csv.reader(f)
            q_mat = np.array([[float(angle) for angle in q]
                              for q in list(reader)])

        with open('{path}/velocities.csv'.format(path=PATH)) as f:
            reader = csv.reader(f)
            qdot_mat = np.array([[float(angle) for angle in q]
                                 for q in list(reader)])

        with open('{path}/effort.csv'.format(path=PATH)) as f:
            reader = csv.reader(f)
            tau_mat = np.array([[float(angle) for angle in q]
                                for q in list(reader)])

        DEBUG and print(t)
        DEBUG and print(q_mat)
        DEBUG and print(qdot_mat)
        DEBUG and print(tau_mat)

        assert len(t) == len(q_mat) == len(qdot_mat) == len(tau_mat)

        # Initialize new ROS node

        rospy.init_node('my_playback')

        pub = rospy.Publisher('/j2s6s200_driver/trajectory_controller/command', JointTrajectory, queue_size=10)

        # Move robot to starting point

        nb = raw_input('Moving robot to START position, press return to start, n to skip')

        if (nb != 'n' and nb != 'N'):
            result = joint_position_client(np.rad2deg(q_mat[0]), prefix)

        # Create trajectory message

        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()

        trajectory.joint_names.append("j2s6s200_joint_1")
        trajectory.joint_names.append("j2s6s200_joint_2")
        trajectory.joint_names.append("j2s6s200_joint_3")
        trajectory.joint_names.append("j2s6s200_joint_4")
        trajectory.joint_names.append("j2s6s200_joint_5")
        trajectory.joint_names.append("j2s6s200_joint_6")

        for i in range(len(t)):
            point = JointTrajectoryPoint()
            point.positions = q_mat[i, :6]
            point.velocities = qdot_mat[i, :6]
            point.effort = tau_mat[i, :6]
            point.time_from_start = rospy.Duration(t[i])
            trajectory.points.append(point)

        # Play trajectory

        nb = raw_input('Starting trajectory playback, press return to start, n to skip')

        if (nb != "n" and nb != "N"):
            pub.publish(trajectory)

        print('Done!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')
