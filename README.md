## Overview

Repository of ROS 2 packages for communication with a 4-DOF robotic finger.\
Contains two packages:
 - finger_manipulation: a driver node for main control functions
 - rqt_finger_manipulation: a customized GUI plugin for controlling with rqt

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
Developed as part of ME 450 at the University of Michigan. Special thanks to our advisors for their support.