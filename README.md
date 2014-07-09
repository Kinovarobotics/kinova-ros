# JACO-ROS

The ``jaco-ros`` module provides a ROS interface for the Kinova Robotics JACO and MICO robotic manipulator arms. This module exposes the Kinova C++ hardware API through ROS. This documentation will usually refer to the JACO arm, but the instructions work with both the JACO and the MICO unless otherwise noted.

## Overview
The JACO API is exposed to ROS using a combination of actionlib (for sending trajectory commands to the arm), services (for instant control such as homing the arm or e-stop) and published topics (joint feedback).  The Arm may be commanded using either angular commands or Cartesian co-ordinates.

In addition, a transform publisher enables visualization of the arm via rviz.

There are three actionlib modules available: ``arm_pose/arm_pose``, ``joint_angles/arm_joint_angles``, and ``fingers/finger_position``.  These server modules accept coordinates which are passed on to the Kinova JACO API for controlling the arm.

The services available are: ``in/home_arm``, ``in/stop``, and ``in/start``.  These services require no input goals, and are intended for quick control of basic arm functions.  When called, home_arm will halt all other movement and return the arm to its "home" position.  The stop service is a software e-stop, which instantly stops the arm and prevents any further movement until the start service is called.

Published topics are: ``out/cartesian_velocity, out/joint_velocity, out/finger_position, out/joint_angles, out/joint_state``, and ``out/tool_position``.  The ``cartesian_velocity`` and ``joint_velocity`` are both subscribers which may be used to set the joint velocity of the arm.  The ``finger_position`` and ``joint_angles`` topics publish the raw angular position of the fingers and joints, respectively, in degrees.  The ``joint_state`` topic publishes via ``sensor_msgs`` the transformed joint angles in radians.  The ``tool_position`` topic publishes the Cartesian co-ordinates of the arm and end effector via ``geometry_msgs``.

## JACO-ROS Module Architecture
The ``jaco_arm_driver`` node acts as an interface between the Kinova JACO C++ API and the various actionservers, message services and topics used to interface with the arm.

The ``jaco_tf_updater`` node subscribes to the jaco_arm_driver node to obtain current joint angle information from the node.  It then publishes a transform which may be used in visualization programs such as rviz.

### Cartesian Control 
Cartesian control is accomplished via actionserver.  The Cartesian co-ordinates are published as a separate topic. The arm has a maximum reach of about 0.9m (90cm), so the "position" range is about +/-0.9 for all three dimensions.  The three wrist joints are capable of continuous rotation, and therefore capable of being commanded up to +/-174.5 rad. You may want to limit to +/-6.28 rad in software to prevent the joints from rotating excessively.

#### Cartesian Control Actionserver Topic, Command, and Parameters

    /jaco_arm_driver/arm_pose/arm_pose

    Message format:
    float64 position.x – end effector distance from the base in meters (left-right relative to base)
    float64 position.y – end effector distance from the base in meters (forward-back relative to base) 
    float64 position.z – end effector distance from the base in meters (up-down relative to base) 
    float64 orientation.x – end effector quaternion orientation 
    float64 orientation.y – end effector quaternion orientation
    float64 orientation.z – end effector quaternion orientation
    float64 orientation.w – end effector quaternion orientation

    Parameters on the parameter server (can be set in the launch file):
    /jaco_arm_driver/arm_pose/tf_prefix – prefix for the tf tree (needed for distinct tf 
        trees with multiple arms)
    /jaco_arm_driver/arm_pose/stall_interval_seconds – duration over which the stall condition is tested
    /jaco_arm_driver/arm_pose/stall_threshold – threshold over which the stall condition is tested 
        (e.g., if there is less than stall_threshold change in measurement over stall_interval_seconds, 
        the action will aborted due to a stall condition)
    /jaco_arm_driver/arm_pose/rate_hz – rate at which the action is tested for completion, stall, or 
        other termination condition
    /jaco_arm_driver/arm_pose/tolerance – tolerance between the measured position and the goal position 
        to complete the action

#### Published Topic for Cartesian Position

    /jaco_arm_driver/out/tool_position

    Message format:
    float64 position.x – end effector distance from the base in meters (left-right relative to base) 
    float64 position.y – end effector distance from the base in meters (forward-back relative to base) 
    float64 position.z – end effector distance from the base in meters (up-down relative to base) 
    float64 orientation.x – end effector quaternion orientation
    float64 orientation.y – end effector quaternion orientation 
    float64 orientation.z – end effector quaternion orientation
    float64 orientation.w – end effector quaternion orientation

### Angular Control 
Angular control is accomplished via actionserver.  The joint angles are published as two separate topics.

All the joints have a range limit.  Joints 1, 4, 5 and 6 have a range of -10,000 to +10,000 degrees.  Joint 2 has a range of +42 to +318 degrees.  Joint 3 has a range of +17 to +343 degrees.  Sending a command past these limits will cause the arm to move to its hard-wired limit, then stop.

#### Angular Control Actionserver Topic, Command, and Parameters

    /jaco_arm_driver/joint_angles/arm_joint_angles

    Message format:
    float32 joint1 – “base” joint angle in radians 
    float32 joint2 – “shoulder” joint angle in radians 
    float32 joint3 – “elbow” joint angle in radians 
    float32 joint4 – first “wrist” joint angle in radians 
    float32 joint5 – second “wrist” joint angle in radians 
    float32 joint6 – “hand” joint angle in radians 

    Parameters on the parameter server (can be set in the launch file):
    /jaco_arm_driver/joint_angles/stall_interval_seconds – duration over which the stall condition is tested
    /jaco_arm_driver/joint_angles/stall_threshold – threshold over which the stall condition is tested 
        (e.g., if there is less than stall_threshold change in measurement over stall_interval_seconds, 
        the action will aborted due to a stall condition)
    /jaco_arm_driver/joint_angles/rate_hz – rate at which the action is tested for completion, stall, or 
        other termination condition
    /jaco_arm_driver/joint_angles/tolerance – tolerance between the measured position and the goal position 
        to complete the action

#### Published Topics for Angular Position in Degrees

    /jaco_arm_driver/out/joint_angles 

    Message format:
    float32 joint1 – “base” joint angle in degrees 
    float32 joint2 – “shoulder” joint angle in degrees
    float32 joint3 – “elbow” joint angle in degrees 
    float32 joint4 – first “wrist” joint angle in degrees
    float32 joint5 – second “wrist” joint angle in degrees
    float32 joint6 – “hand” joint angle in degrees 

#### Published Topics for Angular Position in Radians

    /jaco_arm_driver/out/joint_state 

    Message format:
    string name – array containing the names of the joints 
    float64[] position – array containing joint positions, in transformed radians, of the joints 
    float64[] velocity – array containing the joint velocities (placeholder, contains no data) 
    float64[] effort – array containing the joint forces in newtons (placeholder, contains no data) 

### Finger Control
Finger control is accomplished via actionserver.  The finger angles are published as a separate topic. The range of input for all three fingers on the JACO is approximately 0 (fully open) to 60 (fully closed). The two fingers on the MICO have a range of approximately 0 to 6400.

#### Finger Control Actionserver Topic, Command, and Parameters

    /jaco_arm_driver/fingers/finger_positions

    Message format:
    float32 finger1 – position of finger 1 in degrees 
    float32 finger2 – position of finger 2 in degrees 
    float32 finger3 – position of finger 3 in degrees 

    Parameters on the parameter server (can be set in the launch file):
    /jaco_arm_driver/fingers/stall_interval_seconds – duration over which the stall condition is tested
    /jaco_arm_driver/fingers/stall_threshold – threshold over which the stall condition is tested 
        (e.g., if there is less than stall_threshold change in measurement over stall_interval_seconds, 
        the action will aborted due to a stall condition). This value typically needs to be changed between
        a MICO and a JACO arm.
    /jaco_arm_driver/fingers/rate_hz – rate at which the action is tested for completion, stall, or 
        other termination condition
    /jaco_arm_driver/fingers/tolerance – tolerance between the measured position and the goal position 
        to complete the action. This value typically needs to be changed between
        a MICO and a JACO arm (see the example launch files).

#### Published Topic for Finger Position

    /jaco_arm_driver/out/finger_position 

    Message format:
    float32 finger1 – position of finger 1 in degrees
    float32 finger2 – position of finger 2 in degrees
    float32 finger3 – position of finger 3 in degrees (0 when using the two-fingered MICO)


### Services 
These services may be called at any time to enact basic functions on the arm.  They will override any other actions being carried out by the arm.

#### Homing the Arm 
When called, this service will return the arm to its pre-programmed “home” position.  It is the equivalent of holding down the “home” button on the pendant controller. The service requires no input parameters, and simply reports when the arm has returned home.

    Service Topic:  /jaco_arm_driver/in/home_arm
    Result:  string homearm_result – a string containing the results of the home_arm service

#### Emergency Stop 
When called, this service will immediately stop the arm if it is moving, erase any trajectories still residing in the JACO arm’s FIFO, and enable a software e-stop flag.  This flag will prevent any further movement of the arm, including homing.  Joint angle feedback will continue to function.  The service requires no input parameters.

    Topic:  /jaco_arm_driver/in/stop 
    Result:  string stop_result – a string containing the results of the stop service 

#### Start 
When called, this service will disable the software e-stop flag, and restore control of the arm.  The service requires no input parameters.

    Topic:  /jaco_arm_driver/in/start   
    Result:  string start_result – a string containing the results of the start service 


### Joint Velocity Subscriber
Publishing messages to this topic allows for the arm to be controlled using joint velocity commands. For example, the following command will cause the arm to spin the sixth joint:

    rostopic pub -r 10 /jaco_arm_driver/in/joint_velocity jaco_msgs/JointVelocity 
    "{joint1: 0.0, joint2: 0.0, joint3: 0.0, joint4: 0.0, joint5: 0.0, joint6: 10.0}" 

Note, using tab-completion to create this message will make sure that the command follows the correct syntax and format. If you just copy-and-paste this command into a terminal, it may not work.

### Cartesian Velocity Subscriber
Publishing messages to this topic allows for the arm to be controlled using Cartesian velocity commands. For example, the following command will cause the arm to move in the positive z-direction (up).

    rostopic pub -r 10 /jaco_arm_driver/in/cartesian_velocity geometry_msgs/TwistStamped 
    "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.0, y: 0.0, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" 

Note, using tab-completion to create this message will make sure that the command follows the correct syntax and format. If you just copy-and-paste this command into a terminal, it may not work.

## Launch file 
The jaco_arm.launch should be run prior to using the JACO arm.  It launches two nodes, ``jaco_arm_driver`` and ``jaco_tf_updater``.  These nodes then perform a number of operations that prepare the arm for use.

On launch, the ``jaco_arm_driver`` announces all of the configurations stored in the JACO arm’s permanent memory.  These are settings that, currently, are most easily set using the Windows-only Kinova GUI.  The fingers may move during initialization, but the arm is not automatically homed. If the arm does not respond after initialization, it may need to be homed.

The ``jaco_tf_updater`` begins publishing transform data as soon as it becomes available from the ``jaco_arm_driver node``.


## Installation
To make ros-jaco part of your workspace, follow these steps (assuming your workspace is setup following the standard conventions):

    cd ~/catkin_ws/src
    git clone https://github.com/Kinovarobotics/jaco-ros.git jaco-ros
    cd ~/catkin
    catkin_make

To access the arm via usb copy the udev rule file ``99-jaco-arm.rules`` from ``<your_workspace>/ros-jaco-arm/jaco_driver/udev`` to ``/etc/udev/rules.d/``:

    sudo cp jaco_driver/udev/99-jaco-arm.rules /etc/udev/rules.d/

If you would like the ``jaco_arm_driver`` and ``jaco_tf_updater nodes`` to launch automatically when ROS is started, copy the ``jaco_arm.launch`` file contained in the ``/launch`` folder into the relevant ``/core.d`` folder.


### Using Multiple Arms
This version of ``jaco-ros`` supports multiple arms. In order to use multiple arms you must set the the ``serial_number`` parameter for that arm and a ``tf_prefix`` for both the ``arm_driver`` node and the ``tf_updater`` node. For example, include the following lines in the launch file between ``<node pkg="jaco_driver" type="jaco_arm_driver" ...>`` and ``</node>``:

    <param name="serial_number" value="PJ00123456789012345" />
    <param name="arm_pose/tf_prefix" value="jaco_" />

And the following line in the launch file between ``<node pkg="jaco_driver" type="jaco_tf_updater" ...>`` and ``</node>``:

    <param name="tf_prefix" value="jaco_" />

If no serial number parameter is set, the node will simply try to connect to the JACO or MICO arm that is present.


## Execution 
This package has been tested in Ubuntu 12.04 LTS with ROS Hydro. The 5.01.01 driver/API is included in the package; however, you should ensure that the firmware on the arm is up-to-date. Using old versions of the firmware may result in unexpected behavior.

### Basics
To “home” the arm

    rosservice call jaco/home_arm 

To activate the e-stop (emergency stop) function

    rosservice call jaco/stop 

To restore control of the arm

    rosservice call jaco/start 


### Joint Sensors 
To obtain the raw joint angles in degrees

    rostopic echo jaco/joint_angles 

To obtain the “transformed” joint angles in radians in a standard ``sensor_msgs`` format

    rostopic echo jaco/joint_state 

To obtain the finger angles in degrees

    rostopic echo jaco/finger_position

To obtain the arm’s position in Cartesian units in a standard ``geometry_msgs`` format

    rostopic echo jaco/tool_position 


### Arm Control
Several sample python script action clients are available for manually controlling the arm. These scripts are located in the ``jaco_demo`` package.

To set the joint angles using DH “transformed” angles in radians, use ``joint_angle_workout.py``

    rosrun jaco_demo joint_angle_workout.py node_name random num        - randomly generate num joint angle sets
    rosrun jaco_demo joint_angle_workout.py node_name file_path         - use poses from file
    rosrun jaco_demo joint_angle_workout.py node_name j1 j2 j3 j4 j5 j6 - use these specific angles
    e.g., rosrun jaco_demo joint_angle_workout.py jaco random 10


To set the arm position using Cartesian co-ordinates, use ``cartesian_workout.py``

    rosrun jaco_demo cartesian_workout.py node_name random num          - randomly generate num poses
    rosrun jaco_demo cartesian_workout.py node_name file_path           - use poses from file
    rosrun jaco_demo cartesian_workout.py node_name x y z qx qy qz qw   - use that specific pose
    e.g., rosrun jaco_demo cartesian_workout.py jaco -0.314 -0.339 0.600 -0.591 -0.519 0.324 0.525 

To set the finger positions, use ``gripper_workout.py``

    rosrun jaco_demo gripper_workout.py node_name random num   - randomly generate num poses
    rosrun jaco_demo gripper_workout.py jaco f1 f2 f3          - use that specific pose
    rosrun jaco_demo gripper_workout.py mico f1 f2             - use that specific pose
    e.g., rosrun jaco_demo gripper_workout.py jaco random 10


## Known Limitations
1. The ``joint_state`` topic currently reports only the arm position.  Velocity and Effort are placeholders for future compatibility.

2. When updating the firmware on the arm (e.g., using Jacosoft) the serial number will be set to "Not set" which will cause multiple arms to be unusable. The solution is to make sure that the serial number is reset after updating the arm firmware.

3. After using angular control, the arm may not respond to Cartesian commands (e.g., arm_pose or figer_position) until the arm has been homed.

4. Some virtualization software products are known to work well with this package, while others do not.  The issue appears to be related to proper handover of access to the USB port to the API.  Parallels and VMWare are able to do this properly, while VirtualBox causes the API to fail with a "1015" error.

5. With the latest firmware, the JACO arm will sag slightly when gripper commands are sent. This behavior has not been observed with the MICO arm.

6. Previously, files under ``jaco-ros/jaco_driver/lib/i386-linux-gnu`` had a bug which required uses 32-bit systems to manually copy them into devel or install to work. This package has not been tested with 32-bit systems and this workaround may still be required. 64-bit versions seem to be unaffected.

7. In certain cases, while commanding the MICO arm using angular commands, the force limits may be exceeded and the arm will stop.  If this occurs, consider increasing the force limits of your arm using JacoSoft, or use shorter movements that put less stress on the arm joints.

## Additional Resources
The transformation equations used to convert from the “DH Parameters” to physical angles are listed in the jaco_kinematics.pdf document, included as part of this package.

The Kinova JACO website: http://kinovarobotics.com/products/jaco-research-edition/

## Report a Bug
Any bugs, issues or suggestions may be sent to skynet@clearpathrobotics.com


