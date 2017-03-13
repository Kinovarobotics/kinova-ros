# Gazebo for Kinova robots

## Supported versions
The recommended configuration is ROS Indigo with 64 bit Ubuntu 14.04.

The package may work with other configurations as well, but it has only been tested for the one recommended above. 

# New in this release
- Gazebo support for Jaco, Mico, 7-dof robot
- Gazebo/ros_control parameters added to URDF
- ros_control config files added
- launch files to launch gazebo with ros_control
- script to send trajectory command to robot
- Interface with MoveIt!

# Installation

1. Install Gazebo. For detailed instructions see [install page](http://gazebosim.org/tutorials?tut=ros_installing "http://gazebosim.org/tutorials?tut=ros_installing").  
```
sudo apt-get install ros-indigo-gazebo-ros*  
```
2. Install ros_control [ros_controllers repository](https://github.com/ros-controls/ros_controllers "https://github.com/ros-controls/ros_controllers").  
```
sudo apt-get install ros-indigo-gazebo-ros-control
sudo apt-get install ros-indigo-ros-controllers*
```

3. Checkout kinova-ros in your catkin workspace  
```
git clone https://github.com/Kinovarobotics/kinova-ros
```


# Gazebo/ros_control parameters added to Gazebo

Inertial parameters have been added to all mesh models to support gazebo. Each link has the inertial model for the link and half of the actuator on one side
and half on the other side. COM position and mass of the links are accurate to result in correct torque readings.

The inertia matrix however is a approximation, assuming uniform cylinders for links. More accurate values will be added later.

The joint dynamics parameters, like damping, friction, stiffness, do not accurately represent the hardware.

The joints are configured to have a effortJointInterface.

# ros_control
We use ros_control to control the robot model in Gazebo. See tutorial here - [ros_control tutorial](http://gazebosim.org/tutorials/?tut=ros_control "http://gazebosim.org/tutorials?tut=ros_installing").

The config files for controlling the joints using ros-control have been added for the three robot types.
Three different types of controllers are available - effort, position, velocity.

Trajectory controller has also been added to provide interface for MoveIt!. 

Recommended way to control the robot is to use trajectory controller.

# Launching Gazebo with ros_control
You can launch Gazebo using the launch file -
 
```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
```

The launch file -
1) Launches Gazebo
2) Loads the robot model in Gazebo
3) Launches ros_control controller for the robot

By default the controller is set to launch a trajectory position controller, this is how moveIt! commands the robot.
If you would like to control the joint by joint, set use_trajectory_controller = false

```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300 use_trajectory_controller:= false
```

### Moving the robot
To move the robot in Gazebo using the trajectory controller user can publish jointTrajectory commands to the topic - 
/robot_name/effort_joint_trajectory_controller/command. An example script is available to move the robot to home position.

```
rosrun kinova_control move_robot.py j2n6s300
```

The robot can also be commanded joint by joint through rqt - 

```
roslaunch kinova_control kinova_rqt.launch kinova_robotType:=j2n6s300 
```

For this however, set use_trajectory_controller = false as explained above.

### Interface with MoveIt!
First launch Gazebo - 

```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
```

Next launch moveIt and RViz -

```
roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch
```

You can use the interactive markers in rviz to plan trajectories and when you click execute
trajectories are executed in Gazebo.

You can also run the pick and place demo, more info about it in the MoveIt wiki
[here] (https://github.com/Kinovarobotics/kinova-ros/wiki/MoveIt)  

```
rosrun kinova_arm_moveit_demo pick_place 
```



## Report a Bug
Any bugs, issues or suggestions may be sent to ros@kinovarobotics.com.


