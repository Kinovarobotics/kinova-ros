Teleoperation support for Jaco arm.
==================================
To run the teleoperation node, type simply

	roslaunch jaco_teleop jaco_teleop.launch

or 

	roslaunch jaco_teleop jaco_teleop.launch joystick_type:=xbox_360

if you have the appropriate YAML config file in the `config` directory (in this example `config/xbox_360.yaml`). 

Launching `jaco_teleop.launch` assumes that `jaco_driver/jaco_arm.launch` has already been running.

This node simply remaps gamepad commands to the Jaco dedicated controller commands. Using the default gamepad config, the gamepad is assumed to have two analog joysticks (they produce float values), 4 digital buttons (their output is 0/1) on the left (the cross), 4 digital buttons on the right (ABCD or 1234), two digital buttons on the back side, two analog buttons on the back side, and 4 digital buttons on the "front panel" (Start, Back, Mode, Vibration). Some joysticks can be switched to two modes - in one of them, all 4 back buttons are digital, and in the other one, two of the back buttons are analog. This node needs the analog mode to be set (when using the default gamepad config).
	
Default controls remapping (Logitech F710):
-------------------
		
    Thanks to RIVeR-Lab/wpi_jaco for base of this image

                   Controls                
        up/down                 rotate wrist
       ________                 __________    
      /fin.open\_______________/          \   
     |   _| |_    < >  < home >     (4)    |  
     |  |_   _|   < >    < >     (1)   (3) |  
     |    |_|    ___       ___      (2)    |  
     | fin.close/   \     /   \            |  
     |          \___/     \___/            |  
     |       x/y trans  rotate x/y         |  
     |        _______/---\_______          |  
     |       |                    \        |  
      \      /                     \      /   
       \____/                       \____/    
                                           
    Buttons:                               
      (1) Fingers open
      (2) Fingers close
      (3) Emergency stop                    
      (4) Emergency release

Contributing
------------
If you write your own YAML config for another controller, feel free to fork this repo and place a merge request.