# Main program runners
This package contains the main state machine for the picking challenge.
It also contains other end-to-end demos that integrate multiple parts.

## State machine
Located in `scripts/main.py`.
It is implemented using the [smach](http://wiki.ros.org/smach) framework.
smach has extensive [tutorials](http://wiki.ros.org/smach/Tutorials) on how to use it.

### Getting started
You may need to install [mock](https://pypi.python.org/pypi/mock): `sudo pip install mock`

Launch the appropriate launch file: `roslaunch pr2_pick_main milestone1.launch`, or launch the prereqs and the main file separately:

```
roslaunch pr2_pick_main milestone1_prereqs.launch
python main.py
```

If the robot is busy, you can still run the state machine by mocking out all the
ROS services, publishers, etc. Just run the launch file with `mock:=true`, or with `python main.py --mock`.

## Travel support demo
Picks 3 objects, one from bin G, H, and I.
The robot is assumed to start fairly close up to the shelf and fairly centered.
The order bin is on the ground to the robot's right at the start.

Running:
```
roslaunch pr2_pick_perception perception.launch
roslaunch pr2_moveit_config move_group.launch
roslaunch pr2_pick_main easy_boxes_demo.launch --screen
```

## Manipulation client demos
Demonstrations and example code for the usage of the gripper, torso, and other
clients.