Teleoperation support for Jaco arm.
==================================

Launching jaco_teleop.launch assumes that jaco_driver/jaco_arm.launch has already been running.

This node simply remaps gamepad commands to the Jaco dedicated controller commands. The gamepad is assumed to have two analog joysticks (they produce float values), 4 digital buttons (their output is 0/1) on the left (the cross), 4 digital buttons on the right (ABCD or 1234), two digital buttons on the back side, two analog buttons on the back side, and 4 digital buttons on the "front panel" (Start, Back, Mode, Vibration). Some joysticks can be switched to two modes - in one of them, all 4 back buttons are digital, and in the other one, two of the back buttons are analog. This node needs the analog mode to be set.
	
Controls remapping:
-------------------
		
- Left joystick
 - Translation x/y
 
- Right joystick
 - Rotation x/y
 
- Left back buttons (one analog, one digital)
 - Translation z
 
- Right back buttons (one analog, one digital)
 - Rotation theta (wrist joint)
 
- Left cross up/down
 - Open/close all fingers
 
- A (2), X (1) buttons
 - Open/close all fingers
 
- B (3)
 - Emergency stop (when holding)
 
- Y (4)
 - Release emergency stop
 
- Start button
 - Home arm/go to retreat (it's always needed to home the arm before you can control it)
