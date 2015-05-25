#Gazebo & Jaco/Mico arms

This package provides the basic tools to run the URDF files of Jaco and Mico arms in Gazebo. The controller of the motors are simulated using the [ROS controller manager](http://wiki.ros.org/controller_manager), which manages the following [ROS controllers](http://wiki.ros.org/ros_controllers): JointPositionController and JointTrajectoryController.

The controllers above serve only as a debugging test bed for other algorithms, such as motion planning integrated with visual input, reaching for grasping etc.

## Launching the simulator and loading the world

The global launcher is [gazebo.launch](launch/gazebo.launch), which has defined parameters for the arm, world file and arm pose amongst others. By default the simulator is paused, because the controllers take a little while to start. Thus, after verifying that the controllers are up and running, the play button can be pressed.

## Jaco arm

### Launching the arm in the empty.world

roslaunch jaco_gazebo gazebo.launch robot:=jaco_arm

### Launching the arm in the table_kinect_and_objects.world

roslaunch jaco_gazebo gazebo.launch robot:=jaco_arm world:=table_kinect_and_objects.world pose:="-x 0.0 -y -0.95 -z 1.01 -R 0.0 -P 0.0 -Y 0.0"

##Mico arm

### Launching the arm in the empty.world

roslaunch jaco_gazebo gazebo.launch robot:=mico_arm

### Launching the arm in the table_kinect_and_objects.world

roslaunch jaco_gazebo gazebo.launch robot:=mico_arm world:=table_kinect_and_objects.world pose:="-x 0.0 -y -0.95 -z 1.01 -R 0.0 -P 0.0 -Y 0.0"

## Known limitations

* The gripper of both arms do not behave like the actual grippers
* Masses and inertias need to be updated with their real values
* Joints' stiffness and damping need revision to emulate the arms behavior
