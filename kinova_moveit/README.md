# MoveIt! for Kinova robots

## Supported versions
The recommended configuration is ROS Indigo with 64 bit Ubuntu 14.04.

The package may work with other configurations as well but it only tested for the one recommended above. 

# New in this release
- Robot configs added for Mico, Jaco, and Jaco 7 dof
- IKFast plugins for Mico, Jaco and Jaco 7 dof
- Trajectory following action server
- Trajectory following controller
- Gripper command action server
- Demo for using MoveIt with Kinova arms

# Robot configs for Jaco, Mico, Jaco 7dof

The `kinova-ros` stack provides a ROS interface for the Kinova Robotics JACO, JACO2 and MICO robotic manipulator arms, and it is built to support further kinova products as well. Besides  widely support of Kinova products, there are many bug fixing, improvements and new features as well. The stack is developped upon the Kinova C++ API functions, which communicates with the DSP inside robot base. 


## Installation
To make kinova-ros part of your workspace, follow these steps (assuming your workspace is setup following the standard conventions):

    cd ~/catkin_ws/src
    git clone https://github.com/Kinovarobotics/kinova-ros.git kinova-ros
    cd ~/catkin_ws
    catkin_make

To access the arm via usb copy the udev rule file `10-kinova-arm.rules` from `~/catkin_ws/src/kinova-ros/kinova_driver/udev` to `/etc/udev/rules.d/`:

    sudo cp kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/

## How to use the stack

### launch driver
`kinova_robot.launch` in kinova_bringup folder launches the essential drivers and configurations for kinova robots. kinova_robot.launch has two arguments:

**kinova_robotType** specifies which robot type is used. For better supporting wider range of robot configurations,  *robot type* is defined by a `char[8]`, in the format of: `[{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]`. *Robot category* `{j|m|r|c}` refers to *jaco*, *mico*, *roco* and *customized*, *version* is `{1|2}` for now, *wrist type* `{s|n}` can be spherical or *non-spherical*, *Degree of Freedom* is possible to be `{4|6|7}`, *robot mode* `{s|a}` can be in *service* or *assistive*, *robot hand* `{2|3}` may equipped with *2 fingers* or *3 fingers* gripper. Last two positions are *undifined* and *reserved* for further features.

*eg*: `j2n6a300` (default value) refers to *jaco v2 6DOF assistive 3 fingers*. Please be aware that not all options are valided for different robot types.

**use_urdf** specifies whether the kinematic solution is provided by the URDF model. 

When `use_urdf:=true` (default value), the kinematic solution is automatically solved by URDF model. 
The robot can be virtually presented in the Rviz and the frames in Rviz are located at each joints. 
To visulize the robot in Rviz, run `$ rosrun rviz rviz`, and select *root* as the world frame. 
The robot model will synchronize the motion with the real robot.

If `use_urdf:=false`, the kinematic solution is as ame as the DSP code inside robot. 
Node `kinova_tf_updater` will be activated to publish frames, and the frames are defined 
according the classic D-H converntion(frame may not locat at joints). Even you are not able to visulize
the robot properly in Rviz, you are able to observe the D-H frames in Rviz.

*eg*: `roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n4a200 use_urdf:=true`

If the robot is not able to move after boot, please try to home the arm by either pressing *home* button on the joystick or calling rosservice in the **ROS service commands** below.


## Notes and Limitations
1. Force/torque control is only for advanced users. Please be caution when using force/torque control api functions.

2. The ``joint_state`` topic currently reports only the arm position and
velocity. Effort is a placeholder for future compatibility. Depending on your
firmware version velocity values can be wrong. 

3. When updating the firmware on the arm (e.g., using Jacosoft) the serial number will be set to "Not set" which will cause multiple arms to be unusable. The solution is to make sure that the serial number is reset after updating the arm firmware.

4. Some virtualization software products are known to work well with this package, while others do not.  The issue appears to be related to proper handover of access to the USB port to the API.  Parallels and VMWare are able to do this properly, while VirtualBox causes the API to fail with a "1015" error.

5. Previously, files under ``kinova-ros/kinova_driver/lib/i386-linux-gnu`` had a bug which required uses 32-bit systems to manually copy them into devel or install to work. This package has not been tested with 32-bit systems and this workaround may still be required. 64-bit versions seem to be unaffected.


## Report a Bug
Any bugs, issues or suggestions may be sent to ros@kinovarobotics.com.


