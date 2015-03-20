# Main
This package contains the main state machine for the picking challenge.
It also contains other end-to-end demos that integrate multiple parts.

## State machine
Located in `scripts/main.py`.
It is implemented using the [smach](http://wiki.ros.org/smach) framework.
smach has extensive [tutorials](http://wiki.ros.org/smach/Tutorials) on how to use it.
Our state machine is just a template for right now.

## Travel support demo
Picks 3 objects, one from bin G, H, and I.
The robot is assumed to start fairly close up to the shelf and fairly centered.
The order bin is on the ground to the robot's right at the start.

Running:
```
roslaunch pr2_pick_perception perception.launch
roslaunch pr2_moveit_config move_group.launch
roslaunch pr2_pick_main easy_boxes_demo.launch
```