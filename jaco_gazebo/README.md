#Gazebo & Jaco/Mico arms

This package provides the basic tools to run the URDF files of Jaco and Mico arms in Gazebo. The controller of the motors are simulated using the [ROS controller manager](http://wiki.ros.org/controller_manager), which manages the following [ROS controllers](http://wiki.ros.org/ros_controllers): JointPositionController and JointTrajectoryController.

The controllers above serve only as a debugging test bed for other algorithms, such as motion planning integrated with visual input, reaching for grasping etc.

## Launching the simulator and loading the world

The global launcher is [gazebo.launch](jaco_gazebo/launch/gazebo.launch), which has several parameters for the arm, world file and pose

## Jaco arm

### Launching
Empty world
roslaunch jaco_gazebo gazebo.launch robot:=jaco_arm

##Mico arm

Empty world
roslaunch jaco_gazebo gazebo.launch robot:=mico_arm

World with objects, table and kinect
roslaunch jaco_gazebo gazebo.launch robot:=mico_arm world:=table_kinect_and_objects.world pose:="-x 0.0 -y -0.95 -z 1.01 -R 0.0 -P 0.0 -Y 0.0"


///////////////////////////////////
