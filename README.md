## Overview

Repository of ROS 2 packages for servo motor control of a 16-DOF robotic hand.\
Contains the following two packages:
- mint_hand: this package provides the core communication functions. It includes the `hand_driver` node which wraps the DYNAMIXEL protocol 2.0, plus `pose_record` and `pose_playback` scripts. The pose scripts enable manual programming of continuous motion sequences into the robot by manipulating the finger joints. The hand_driver node offers the following services and topics, addressed by motor ID:
	* `get_current`: service to read present current in mA being consumed by a motor
	* `get_position`: service to read present raw encoder position
	* `get_position_limits`: service to read min and max encoder limits set for a motor
	* `get_temperature`: service to read present internal temperature in Celsius
	* `get_torque_enabled`: service to see whether a motor can presently receive current
	* `set_torque_enabled`: service to enable/disable current supply
	* `set_operating_mode`: service to switch between position and current-based operation
	* `goal_position`: topic to publish encoder positions for a motor to move to
    * `goal_current`: topic to publish current values in mA to supply to a motor  
  \+ bulk control variants for `get_current`, `get_position`, `get_temperature`, and `goal_position`
- rqt_mint_hand: this package offers a GUI for real-time robot monitoring, control and diagnostics. Can be run standalone or loaded as an rqt plugin.
<p align="center">
  <img src="https://github.com/Yuandi-Huang/mint_hand/blob/main/media/full_hand_gui.gif" 
  alt="GIF of robot control using GUI"/>
</p>
<p align="center">
  <img src="https://github.com/Yuandi-Huang/mint_hand/blob/main/media/grab_banana.gif" 
  alt="GIF of robot grabbing banana"/>
</p>

## Dependencies
- [ROS 2](https://docs.ros.org/en/jazzy/Installation.html)
- [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/)

## Installation:
```
git clone https://github.com/Yuandi-Huang/mint_hand.git
colcon build --packages-select mint_hand
colcon build --packages-select rqt_mint_hand
source install/setup.bash
```
## Acknowledgments
Developed at the University of Michigan. Special thanks to the MMINT Lab for their support.