#! /usr/bin/env python
"""A helper program to test gripper goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys

import actionlib
import jaco_msgs.msg

import goal_generators


def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + str(sys.argv[1]) + '_arm_driver/fingers/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])

    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the gripper action timed-out')
        return None


if __name__ == '__main__':
    if (len(sys.argv) not in [4, 5] or
            'help' in str(sys.argv) or
            str(sys.argv[1]) not in ['jaco', 'mico']):

        print('Usage:')
        print('    gripper_workout.py jaco random num   - randomly generate num poses')
        print('    gripper_workout.py jaco f1 f2 f3     - use that specific pose')
        print('    gripper_workout.py mico f1 f2        - use that specific pose')
        exit()

    try:
        rospy.init_node(str(sys.argv[1]) + '_gripper_workout')

        if str(sys.argv[2]) == 'random' and len(sys.argv) == 4:
            print('Using {} randomly generated finger positions'.format(int(sys.argv[3])))
            if str(sys.argv[1]) == 'jaco':
                positions = goal_generators.random_jaco_finger_positions(int(sys.argv[3]))
            else:
                positions = goal_generators.random_mico_finger_positions(int(sys.argv[3]))
        elif str(sys.argv[1]) == 'jaco' and len(sys.argv) == 5:
            print('Using the specified JACO finger positions:')
            raw_positions = [float(n) for n in sys.argv[2:]]
            positions = [raw_positions]
        elif str(sys.argv[1]) == 'mico' and len(sys.argv) == 4:
            print('Using the specified MICO finger positions:')
            raw_positions = [float(n) for n in sys.argv[2:]]
            positions = [raw_positions]
        else:
            print('Could not parse arguments, use gripper_workout.py help to see examples')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            exit()

        for position in positions:
            print('    position: {}'.format(position))
            result = gripper_client(position)

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
