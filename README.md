# Amazon Picking Challenge
This is the main code repository for the UW Amazon Picking Challenge team.

## Packages
### pr2_pick
A metapackage for all the other packages.

### pr2_pick_smach
The main state machine for the picking challenge.

### pr2_pick_perception
- Contains a launch file which starts the Kinect.
- Contains a script for recording point cloud data.
- Contains a "mock perception" class for testing purposes.

### pr2_pick_msgs
Contains common message types for the picking challenge.

### pr2_pick_manipulation
- Contains demo code for picking 3 items out of the bin.
- Contains gripper open/close C++ client
- Contains base movement / odometry C++ client