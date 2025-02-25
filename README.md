## Overview

Repository of ROS 2 packages for communication with a 4-DOF robotic finger.\
Contains two packages:
- finger_manipulation: a driver node for main control functions. It contains the following services and topics, addressed by motor ID:
	* get_current: service to read present current in mA being consumed by a motor
	* get_position: service to read present raw encoder position
	* get_position_limits: service to read min and max encoder limits set for a motor
	* get_temperature: service to read present internal temperature in Celsius
	* get_torque_enabled: service to see whether a motor can presently receive current
	* set_torque_enabled: service to enable/disable current supply
	* set_operating_mode: service to switch between position and current-based operation
	* goal_position: topic to publish encoder positions for a motor to move to
	* goal_current: topic to publish current values in mA to supply to a motor


- rqt_finger_manipulation: a custom GUI plugin for real-time monitoring, control and diagnostics through rqt.

## Dependencies
- [ROS 2](https://docs.ros.org/en/jazzy/Installation.html)
- [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/)

## Installation:
```
git clone https://github.com/Yuandi-Huang/finger_manipulation.git
colcon build --packages-select finger_manipulation
colcon build --packages-select rqt_finger_manipulation
source install/setup.bash
```
## Acknowledgments
Developed at the University of Michigan. Special thanks to the MMINT Lab for their support.