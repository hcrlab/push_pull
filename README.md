# Amazon Picking Challenge
This is the main code repository for the UW Amazon Picking Challenge team.

## Getting started
```
cd ~/catkin_ws_hydro/src
git clone --recursive git@gitlab.cs.washington.edu:amazon-picking-challenge-2015/pr2_pick.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=hydro -y
catkin_make
```

## Packages
### festival_tts
A text-to-speech node.
It's useful for debugging purposes to have the robot say stuff aloud as it works.

### joint_states_listener
A service that gets the most recent joint states message.

### pr2_ethercat_drivers
- Contains firmware for the fingertip optical sensor.

### pr2_pick
A metapackage for all the other packages.

### pr2_pick_contest
Contest interface, inventory management, and strategy.

### pr2_pick_main
A package that contains the main state machine runner for the picking challenge, as well as other end-to-end demos.
Test programs for specific components might be in other packages, but end-to-end integration stuff should go here.

- Contains a template of the main state machine.
- Contains demo code for picking 3 items out of the bin.

### pr2_pick_manipulation
- Contains services for controlling the robot (torso, base, grippers, etc.)
- Contains a launch file for the above services.
- Contains C++ libraries for controlling the robot.
- Contains a tool for reading the current end-effector pose, using your phone.

### pr2_pick_msgs
Contains common message types for the picking challenge.

### pr2_pick_perception
- Contains a launch file which starts the Kinect.
- Contains a script for recording point cloud data.
- Contains a "mock perception" class for testing purposes.

### pr2_pretouch_optical_dist
- Contains a launch file for running the fingertip optical sensor.
- Contains a visualizer for the fingertip optical sensor data.

### trajopt_test
- Contains a service that uses trajopt to do motion planning and trajectory execution.

## Style guides
These are just suggested:
- [Google C++ Style Guide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html)
- [Python PEP8](https://www.python.org/dev/peps/pep-0008/)

## Fingertip optical sensor
We have installed an optical sensor on one of the PR2's fingertips.
[Full documentation for the fingertip sensor](https://bitbucket.org/uwsensors/pr2_pretouch_optical_dist/wiki/Publishing%20Distance%20Data).

- Must use Hydro.
- Whenever the robot is power-cycled, you must re-upload some firmware to the robot.
- This launch file starts the optical distance publisher: `roslaunch pr2_pretouch_optical_dist optical_dist.launch`
- The data can be visualized using `rosrun pr2_pretouch_optical_dist visualizer.py`