# Amazon Picking Challenge
This is the main code repository for the UW Amazon Picking Challenge team.

## Getting started
```
sudo pip install mock
sudo apt-get install ros-hydro-cmake-modules

cd ~/catkin_ws_hydro/src
git clone --recursive git@gitlab.cs.washington.edu:amazon-picking-challenge-2015/pr2_pick.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=hydro -y
catkin_make
```

If you have a problem with catkin not finding `pr2_pretouch_optical_dist`, then from the root of the repository, run:
```
git submodule init
git submodule update
```

## Running the code
[Recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md)

```bash
# On a desktop computer
setrobot c1 # Set ROS_MASTER_URI, see recommended .bashrc
roslaunch pr2_pick_main rviz.launch

# On hal9k
# hal9k is the laptop in the robot's backpack.
# We run MoveIt on hal9k because it runs too slowly on c1.
# You must launch MoveIt first, or else the prerequisites below won't work.
ssh c1
ssh jstn@hal9k # SSH into hal9k via c1
amazon # Sets the ROS_MASTER_URI to http://c1:11311
roslaunch pr2_pick_manipulation move_group.launch

# On the robot
roslaunch pr2_pick_main main_prereqs.launch
rosrun pr2_pick_main main.py
```

To see a visualization of the execution, open rviz and use the config file in `pr2_pick/config`

## Packages

- **festival_tts**:
A text-to-speech node.
It's useful for debugging purposes to have the robot say stuff aloud as it works.

- **pr2_ethercat_drivers**:
Contains firmware for the fingertip pressure and optical sensor.

- **pr2_pick**:
A metapackage for all the other packages.
Contains the rviz config file.

- **pr2_pick_contest**:
Contains inventory management services.

- **pr2_pick_main**:
Contains the main state machine, and implementation for all the states.

- **pr2_pick_manipulation**:
Contains services and libraries for controlling the robot, planning with the arms, and reading off the end-effector pose.

- **pr2_pick_perception**:
Contains services and libraries for localizing the shelf, analyzing point clouds, publishing static TFs, and recording data.

- **pr2_pretouch_optical_dist**:
Contains launch files and visualizers for the fingertip optical sensor.

- *joint_states_listener*:
Unused.
A service that gets the most recent joint states message.

- *pr2_pick_msgs*:
Unused, put messages in the related package instead.

- *trajopt_test*:
Unused.
A service that moves the arm using trajopt.

## Backpack computer network configuration
See [Backpack configuration](https://github.com/hcrlab/wiki/blob/master/pr2/backpack_configuration.md)

## Style guides
These are just suggested:
- [Google C++ Style Guide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html)
- [Python PEP8](https://www.python.org/dev/peps/pep-0008/)

You can read about [auto code formatting](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/auto_code_formatting.md) if you're interested.

## Pressure sensor
We have installed two pressure sensors on the robot's right hand.
To upload the firmware, go to `pr2_ethercat_drivers/wg006_fingertip_dev` and run `upload_pressure_firmware.sh`

You should be able to specify the max_effort for the right hand gripper actions and have it work appropriately.

## Fingertip optical sensor
We have installed an optical sensor on one of the PR2's fingertips.
[Full documentation for the fingertip sensor](https://bitbucket.org/uwsensors/pr2_pretouch_optical_dist/wiki/Publishing%20Distance%20Data).

- Must use Hydro.
- Whenever the robot is power-cycled, you must re-upload some firmware to the robot.
- This launch file starts the optical distance publisher: `roslaunch pr2_pretouch_optical_dist optical_dist.launch`
- The data can be visualized using `rosrun pr2_pretouch_optical_dist visualizer.py`