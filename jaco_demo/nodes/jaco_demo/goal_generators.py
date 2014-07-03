import numpy as np


def random_pose_generator(n_poses=1):
    """A generator that yields random poses, position: [x,y,z] orientation: [x,y,z,w]."""
    for i in xrange(n_poses):
        orient = np.random.random(4)
        mag = np.sqrt(sum(np.power(orient, 2)))
        yield np.random.random(3), orient / mag


def poses_from_file(filepath):
    """A generator that yields poses from file, position: [x,y,z] orientation: [x,y,z,w]."""
    with open(filepath) as f:
        for line in f.readlines():
            if len(line) > 10:
                vals = [float(n) for n in line.strip('\n').split(' ')[:7]]
                mag = np.sqrt(sum(np.power(vals[3:], 2)))
                yield vals[:3], vals[3:] / mag


def random_joint_angles_generator(n_poses=1):
    """A generator that yields random joint angles."""
    for i in xrange(n_poses):
        # TODO: Check that the angles are valid
        # 1: anything
        # 2: -1.571 +/- 1
        # 3: -1.571 +/- 1
        # 4-6: anything
        offset = np.array([0.0, -0.5 * np.pi, -0.5 * np.pi, 0.0, 0.0, 0.0])
        scale = np.array([2 * np.pi, 2.1, 3.2, 2 * np.pi, 2 * np.pi, 2 * np.pi])
        yield (np.random.random(6) - 0.5) * scale + offset


def joint_angles_from_file(filepath):
    """A generator that yields joint angles from file."""
    with open(filepath) as f:
        for line in f.readlines():
            if len(line) > 10:
                yield [float(n) for n in line.strip('\n').split(' ')[:6]]


def random_jaco_finger_positions(n_positions=1):
    """ """
    for i in xrange(n_positions):
        # finger min/max range: [0.25 to 56]?
        yield np.random.random(3) * 55.75 + 0.25


def random_mico_finger_positions(n_positions=1):
    """ """
    for i in xrange(n_positions):
        # finger min/max range: [12 to 6450]?
        yield np.random.random(2) * 6300.0 + 12.0


if __name__ == '__main__':

    print('You probably want to include these functions in a larger project')
    print('rather than executing this file on its own, but here are some ')
    print('things these functions can do!\n')

    print('Use random_pose_generator() to generate five random poses:')
    for position, orientation in random_pose_generator(5):
        print('    position: {}, orientation: {}'.format(position, orientation))
    print('')

    print('Use poses_from_file() to load a predefined sequence of poses:')
    for position, orientation in poses_from_file('pose_sequences/demo_pose_sequence.txt'):
        print('    position: {}, orientation: {}'.format(position, orientation))
    print('')

    print('Use random_joint_angles_generator() to generate five random sets of joint angles:')
    for angle_set in random_joint_angles_generator(5):
        print('    angles: {}'.format(angle_set))
    print('')

    print('Use joint_angles_from_file() to load a predefined sequence of poses:')
    for angle_set in joint_angles_from_file('joint_sequences/demo_joint_sequence.txt'):
        print('    angles: {}'.format(angle_set))
    print('')

    print('Use random_finger_positions() to generate five random sets of finger positions:')
    for positions in random_jaco_finger_positions(5, n_fingers=3):
        print('    positions: {}'.format(positions))
    print('')