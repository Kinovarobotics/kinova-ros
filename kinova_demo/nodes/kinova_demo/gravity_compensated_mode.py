#! /usr/bin/env python3
"""A test program to test action servers for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')

from kinova_msgs.srv import *
from robot_control_modules import *

prefix = 'j2s7s300_'
nbJoints = 7
interactive = True
duration_sec = 100


if __name__ == '__main__':
  try:        
    prefix, nbJoints = argumentParser(None)
    rospy.init_node('torque_compensated_mode')
    if (interactive == True):
      nb = input("Moving robot to candle like position, and setting zero torques, press return to start, n to skip")
    if (nb != "n" and nb != "N"):

      result = joint_position_client([180]*7, prefix)

      if (interactive == True):
        nb = input('Setting torques to zero, press return')
      ZeroTorque(prefix)

    if (interactive == True):
      nb = input('Starting gravity compensation mode')

      publishTorqueCmd([0,0,0,0,0,0,0], duration_sec, prefix)

    print("Done!")
  except rospy.ROSInterruptException:
    print("program interrupted before completion")
