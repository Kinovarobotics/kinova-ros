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


def new_trajectory_msg(t, q_mat, qdot_mat):
    msg_trajectory = JointTrajectory()
    msg_trajectory.header.stamp = rospy.Time.now()

    for i in range(1, nbJoints + 1):
        msg_trajectory.joint_names.append('{}joint_{}'.format(prefix, i))

    DEBUG and print(msg_trajectory.joint_names)

    for i in range(len(t)):
        point = JointTrajectoryPoint()
        point.positions = q_mat[i, :6]
        point.velocities = qdot_mat[i, :6]
        point.time_from_start = rospy.Duration(t[i])
        msg_trajectory.points.append(point)

    return msg_trajectory


if __name__ == '__main__':
    try:
        # Parse command line arguments
        prefix, nbJoints = argumentParser(None)

        # Read motion

        t = None
        q_mat = None
        qdot_mat = None
        seed_q_mat = None
        seed_qdot_mat = None

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

        with open('{path}/seed_positions.csv'.format(path=PATH)) as f:
            reader = csv.reader(f)
            seed_q_mat = np.array([[float(angle) for angle in q]
                                   for q in list(reader)])

        with open('{path}/seed_velocities.csv'.format(path=PATH)) as f:
            reader = csv.reader(f)
            seed_qdot_mat = np.array([[float(angle) for angle in q]
                                      for q in list(reader)])

        DEBUG and print(t)
        DEBUG and print(q_mat)
        DEBUG and print(qdot_mat)
        DEBUG and print(seed_q_mat)
        DEBUG and print(seed_qdot_mat)

        assert len(t) == len(q_mat) == len(qdot_mat) == len(seed_q_mat) == len(seed_qdot_mat)

        # Initialize new ROS node
        rospy.init_node('my_playback')
        pub = rospy.Publisher('/j2s6s200_driver/trajectory_controller/command', JointTrajectory, queue_size=10)

        # Play seed
        msg_trajectory = new_trajectory_msg(t, seed_q_mat, seed_qdot_mat)
        nb = raw_input('Starting SEED playback, press return to start, n to skip')
        if (nb != 'n' and nb != 'N'):
            joint_position_client(np.rad2deg(seed_q_mat[0]), prefix)
            pub.publish(msg_trajectory)

        # Play trajectory
        msg_trajectory = new_trajectory_msg(t, q_mat, qdot_mat)
        nb = raw_input('Starting trajectory playback, press return to start, n to skip')
        if (nb != 'n' and nb != 'N'):
            joint_position_client(np.rad2deg(q_mat[0]), prefix)
            pub.publish(msg_trajectory)

        print('Done!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')
