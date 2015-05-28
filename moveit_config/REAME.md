#MoveIt! & Jaco/Mico arms

This folder contains the generated files from MoveIt! setup assistant in ROS Indigo. There is a launcher in each folder that runs with the robot (simulated in gazebo or the actual one).

## Jaco arm

### Launching the motion planner using fake trajectory execution
    roslaunch jaco_arm_gazebo_config demo.launch

### Launching the motion planner using gazebo
    rosrun jaco_gazebo jaco_gazebo_follow_trajectory_action_server.py jaco
    roslaunch jaco_arm_gazebo_config moveit.launch

##Mico arm

### Launching the motion planner using fake trajectory execution with Rviz markers at the tip
    roslaunch mico_arm_gazebo_config demo.launch

### Launching the motion planner using gazebo with Rviz markers at the tip
    rosrun jaco_gazebo jaco_gazebo_follow_trajectory_action_server.py mico
    rosrun jaco_gazebo gripper_action_server.py
    roslaunch mico_arm_gazebo_config moveit.launch

### Launching the motion planner using the real robot with Rviz markers at the tip
    roslaunch mico_arm_config moveit.launch

### Launching the motion planner using the real robot
    roslaunch mico_arm_and_gripper_gazebo_config moveit.launch

