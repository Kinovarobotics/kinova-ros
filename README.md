# uml_hri_nerve_pick_and_place

## This README will be updated and cleaned up in the near future
This was put together without going crazy in-depth but making sure that everything works enough to get a sort fo starting point established. There will be a lot more done to this package as well as numerous edits made to the base files that are already included and the README. I apologize for the imcompleteness but it took a bit longer to get this slapped together than I anticipated and I didn't want to delay it any longer.

Next step #TODO is: 

1. include more generalized C++ and Python code for controlling the robots
2. attach gripper/use correct launch file args to attach gripper to Gen3 robot
3. more specific instructions on controlling the Jaco2 gripper
4. set up some default transforms for the 4 cameras on the workstation
5. put in some simple PCL and OpenCV code to play around with
6. set up an OOP code structure for students to work off of
7. vastly improve 3d models
8. add simple functional pick and place demo code (C++ and Python)
9. definitely more things I'll think of along the way

This is mostly for me but feel free to look into any of these things in the time that I am away

## Precursory instructions 
In order to use this package, you will need Moveit! installed  
http://docs.ros.org/kinetic/api/moveit\_tutorials/html/doc/getting\_started/getting\_started.html  

You will also need to be able to use Realsense Cameras (specifically the d435i)  
The instructions for this are currently contained within the ur5e\_workstation package (flynn\-nerve/ur5e\_workstation/scripts)  
A script will eventually be put into this package or somewhere more useful but for now those who need it already have it installed  

You will also need the ros\_kortex (Gen3 control) package and the kinova\_ros (Jaco2 control) package  

## Steps for specific installations
You should install both of the following packages even if you only intend to use one  

### Installing ros_kortex package:
(https://github.com/Kinovarobotics/ros_kortex)  
As always, checkout the package on Github for more information (click on the wiki link along the top for guides if there are any)  

sudo apt install python3 python3-pip  
sudo python3 -m pip install conan  
sudo pip3 install --upgrade jinja2  
conan config set general.revisions_enabled=1  
conan profile new default --detect > /dev/null  
conan profile update settings.compiler.libcxx=libstdc++11 default  

git clone -b kinetic-devel https://github.com/Kinovarobotics/ros_kortex.git  
catkin build  
rosdep install --from-paths src --ignore-src -y  
  
### Installing kinova_ros package:
(https://github.com/Kinovarobotics/kinova-ros/wiki/Gazebo)  
As always, checkout the package on Github for more information (click on the wiki link along the top for guides if there are any)  
  
sudo apt-get install ros-kinetic-gazebo-ros-control  
sudo apt-get install ros-kinetic-ros-controllers*  
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin  

git clone -b master https://github.com/Kinovarobotics/kinova-ros  
catkin build  
rosdep install --from-paths src --ignore-src -y  

## Here's some more useful information you may find yourself wanting
This can be extrapolated to other hardware but it gives you an idea of how to control ne of the grippers  
Hint: look at your topics and try to figure out how to manually open and close a gripper  
The next step is to make calls to the action servers that control the gripper on your robot (GripperAction)  

### Gripper Control information for the Gen3 Robot

------------------------------------------------------------------------------------------

gripper control from terminal example:  
rostopic pub /my_gen3/robotiq_2f_85_gripper_controllerripper_cmd/goal control_msgs/GripperCommandActionGoal '{goal: {command: {position: 1, max_effort: 10}}}'  

** position goes from 0 (open) to 0.800 (closed)  
** max_effort will need to be explored later  

-------------------------------------------------------------------------------------------

gripper control topic and message type:  
rostopic info /my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd/goal  
Type: control_msgs/GripperCommandActionGoal  

Publishers:  
 * /my_gen3/move_group  

Subscribers:  
 * /gazebo  

-------------------------------------------------------------------------------------------

gripper control message type info:  
```
rosmsg show control_msgs/GripperCommandActionGoal  
std_msgs/Header header  
  uint32 seq  
  time stamp  
  string frame_id  
actionlib_msgs/GoalID goal_id  
  time stamp  
  string id  
control_msgs/GripperCommandGoal goal  
  control_msgs/GripperCommand command  
    float64 position  
    float64 max_effort  
```

### Workstation Camera Topics

There are four (4) cameras attached to the workstation currently  
They are labeled from the robot's perspective:  

1. left\_camera
2. right\_camera
3. rear\_camera
4. top\_camera

The cameras are simulated within the Gazebo environment and as such are not viewable in Rviz (until you publish transforms for them!)  
You can use the following command to bring up a useful camera viewer GUI:  

rosrun rqt\_image\_view rqt\_image\_view  

## Workstation Jaco2 Sim

With no sim workstation:  
1st Terminal: roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2s7s300  
2nd Terminal: roslaunch j2s7s300_moveit_config j2s7s300_gazebo_demo.launch  

With sim workstation:
1st Terminal: roslaunch uml_hri_nerve_pick_and_place robot_launch.launch kinova_robotType:=j2s7s300  
2nd Terminal: roslaunch uml_hri_nerve_pick_and_place j2s7s300_gazebo_workstation_demo.launch  

## Workstation Gen3 Sim

With no sim workstation:  
roslaunch kortex_gazebo spawn_kortex_robot.launch  

With sim workstation:  
roslaunch uml_hri_nerve_pick_and_place gen3_gazebo_workstation.launch  
