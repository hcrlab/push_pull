# Amazon Picking Challenge
This is the main code repository for the UW Amazon Picking Challenge team.

## Packages
### joint_states_listener
A service that gets the most recent joint states message.

### pr2_pick
A metapackage for all the other packages.

### pr2_pick_main
A package that contains the main state machine runner for the picking challenge, as well as other end-to-end demos.
Test programs for specific components might be in other packages, but end-to-end integration stuff should go here.

- Contains demo code for picking 3 items out of the bin.

### pr2_pick_manipulation
- Contains gripper open/close C++ client
- Contains base movement / odometry C++ client
- Contains arm navigation C++ interface, and implementations using MoveIt and trajopt.

### pr2_pick_msgs
Contains common message types for the picking challenge.

### pr2_pick_perception
- Contains a launch file which starts the Kinect.
- Contains a script for recording point cloud data.
- Contains a "mock perception" class for testing purposes.

### trajopt_test
- Contains a service that uses trajopt to do motion planning and trajectory execution.