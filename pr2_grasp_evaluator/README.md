# Instructions

```
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_pick_manipulation move_group.launch
roslaunch pr2_grasp_evaluator prereqs.launch
rosrun pr2_grasp_evaluator evaluate_grasps_node.py
``` 
