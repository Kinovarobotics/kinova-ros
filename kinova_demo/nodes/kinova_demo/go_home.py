#! /usr/bin/env python

import numpy as np
import roslib
import rospy
from robot_control_modules import *
roslib.load_manifest('kinova_demo')

prefix = 'j2s6s200'
nbJoints = 6

if __name__ == '__main__':
    try:
        prefix, nbJoints = argumentParser(None)

        rospy.init_node('go_home')

        result = joint_position_client([180, 180, 180, 180, 180, 180, 0, 0], prefix)

        print("Done!")

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
