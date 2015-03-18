# pr2_pick_manipulation

Contains code for PR2 manipulation and robot control.

## Libraries
### gripper.h
C++ action client wrapper for opening and closing the gripper.

Sample usage:
```cpp
Gripper right_gripper("r_gripper_controller/gripper_action");
right_gripper.open();
right_gripper.close();
```
