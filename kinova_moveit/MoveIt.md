# MoveIt! for Kinova robots

## Supported versions
The recommended configuration is ROS Indigo with 64 bit Ubuntu 14.04.

The package may work with other configurations as well, but it has only been tested for the one recommended above. 

# New in this release
- Robot configs added for Mico, Jaco, and Jaco 7 dof
- IKFast plugins for Mico, Jaco and Jaco 7 dof
- Trajectory following action server
- Trajectory following controller
- Gripper command action server
- Sample pick and place code demonstrating use of MoveIt with all Kinova arms - demo includes adding
  obstacles, ataching obstacles to robot, adding constraints and test motion plans

# Installation

1. Install MoveIt! For detailed instructions see [MoveIt install page](http://moveit.ros.org/install/ "http://moveit.ros.org/install/").  
```
sudo apt-get install ros-indigo-moveit
```
2. Install Trac_IK see [Trac_IK repository](https://bitbucket.org/traclabs/trac_ik "https://bitbucket.org/traclabs/trac_ik").  
```
sudo apt-get install ros-indigo-trac-ik
```
3. Checkout kinova-ros in your catkin workspace  
```
git clone https://github.com/Kinovarobotics/kinova-ros
```


# MoveIt config files for Kinova robots

MoveIt! setup assistant is the easiest way to quickly setup robot configs for your robot.
You can find the tutorial to use it [here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html "http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html").

Config files for Kinova robots (Jaco, Mico, and Jaco-7-dof) have already been set up, the
config folders can be found at ```kinova-ros/kinova_moveit/robot_configs/```  

### Move groups
The robots have been setup to use two move groups - 1) arm 2) gripper

[[images/jaco_move_groups.png]]

# Interacting with Robot using MoveIt RViz plugin

Easiest way to start moving the robot with MoveIt is with the moveIt! rviz plugin. 

### Launching MoveIt! and RViz
You can choose launch MoveIt! with a virtual robot, useful for visualization and testing.
Or you can choose to launch MoveIt! with the kinova_driver node which controls the actual
robot.

You can also launch MoveIt with Gazebo and ros_control.
 
#### With virtual robot (MoveIt Fake Controller) 
```
roslaunch robot_name_moveit_config robot_name_virtual_robot_demo.launch
```

For eg, for Jaco robot -
 
```
  roslaunch j2n6s300_moveit_config j2n6s300_virtual_robot_demo.launch
```

Launches the -
 
- move_group node, with
  - fake controllers, defined in /config/fake_controllers.yaml
- fake joint_state_publisher, robot_state_publisher
- rviz

This configuration will let you play with the virtual robot and test your MoveIt envirionment
without needing the real robot.

#### With actual robot connected

```
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=*robot_type*
```

Launches the kinova_driver node to control the arm.

```
roslaunch robot_name_moveit_config robot_name_demo.launch
```

Launches the - 

- move_group node, with 
  -  controllers, defined in /config/controllers.yaml 
- joint_trajectory_action_server node available in kinova_driver
- gripper_command_action_server node available in kinova_driver
- rviz 

This configuration will move the actual robot, so be careful before you execute your trajectories.

#### With Gazebo and ros_control

This is explained in the Gazebo wiki [here] (https://github.com/Kinovarobotics/kinova-ros/wiki/Gazebo) 

### RViz MoveIt Plugin

The image below shows the RViz window with the MoveIt! plugin. You can move the robot's end-effector
using the interactive markers (marked in the image by a red rectangle). While you move the end-effector
MoveIt! runs inverse kinematics to update the joint positions while you drag the marker.

[[images/jaco_rviz_moveIt.png]]

### Setting Start and Goal poses

The planning tab (marked in the image by a blue rectangle), gives you the ability to plan and execute
trajectory. To do this drag the interactive marker to the starting pose that you would like to set for the
robot (if you are connected to the real robot, keep the start state as the current state). In the query 
menu set the start state. Next follow the same procedure to set the goal state. 

### Planning and executing trajectories

Next, click on *plan* and MoveIt will visualize its planned path to move from the start state to the goal 
state. If the visualized plan is acceptable you can command the robot to execute the plan by clicking 
*execute*. If you are connected to the real robot the robot will move according to the visualized plan,
or if you are running the virtual robot the joints of the model will move to the goal state without
the real robot moving.

The moveIt! RViz plugin can also be used to add obstacles, edit planning parameters. It is left to 
the users to explore these features on their own. We will demonstrate these features through the
MoveIt API.


# Interacting with Robot using MoveIt API
MoveIt repository has examples for using its API. A pick and place demo has been adapted for 
Kinova robots to help users develop their own applications.

### Pick and place demo

To run the demo, launch moveIt and RViz -

```
roslaunch robot_name_moveit_config robot_name_virtual_robot_demo.launch
```

or with real robot connected - 

```
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=*robot_type*
```

Next run the pick and place demo node - 

```
rosrun kinova_arm_moveit_demo pick_place 
```

The script runs through various senarios -

- Setting up the workscene, adding obstacles
- Setting goals for the robot in Joint space
- setting goals in Cartesian space
- Setting path constraints
- Attaching objects to robot 

Attaching objects is useful when the robot is holding objects in its gripper and it needs
to move the object through the workspace while checking for collisions for the robot and 
the object it is holding.


# Kinematics

Three types of kinematics plugins have been test with this package - 

1. Kinematics and Dynamics Library (KDL)
2. IKFast
3. Trac_IK

It is possible to set which library to use with MoveIt by setting the kinematics_solver 
parameter in the file robot_config/config/kinematics.yaml.

The recommended kinematics plugin to use is **Trac_IK** with parameter solve type set to
**Distance** or **Manipulation2**.  
**Note** - To make sure the parameter is correctly set, the demo launch file sets 
the parameter '/pick_place/solve_type' in the parameter server.

**IKFast** works well too, unfortunately, it often gives the elbow down solution which is
undesireble. This will be improved with future releases.

**KDL** has a higher faliure rate than the other two.  


## Report a Bug
Any bugs, issues or suggestions may be sent to ros@kinovarobotics.com.


