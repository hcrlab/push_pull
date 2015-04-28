# Main program runners
This package contains the main state machine for the picking challenge.
It also contains other end-to-end demos that integrate multiple parts.
The state machine is located in `scripts/main.py`.
It is implemented using the [smach](http://wiki.ros.org/smach) framework.
smach has extensive [tutorials](http://wiki.ros.org/smach/Tutorials) on how to use it.

## Running the code
[Recommended .bashrc](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/recommended_bashrc.md)

```bash
# On a desktop computer
setrobot c1 # Set ROS_MASTER_URI, see recommended .bashrc
roslaunch pr2_moveit_config move_group.launch
rviz # Use the config file in pr2_pick/config

# On the robot
# Launch move_group.launch first, otherwise the MoveIt service will crash.
roslaunch pr2_pick_main main_prereqs.launch
python main.py --debug
```

To see a visualization of the execution, open rviz and use the config file in `pr2_pick/config`

## Adding a new service / publisher / action client to the state machine.
1. Edit the appropriate launch files so that your service is brought up.
2. Open `scripts/state_machine_factory.py`
3. Add your service to `real_robot_services`.
   You can ignore the mock services.
4. Inside a state where you'd like to use the service, add to the `__init__` method:
   `self._do_something = kwargs['do_something']`
5. When you want to call your service, use the pattern:
   ```
   self._do_something.wait_for_service()
   response = self._do_something(...)
   ```

## Adding a new visualization
1. Add marker code to `visualization.py`.
   Your code should accept a marker publisher or an interactive marker server as a parameter.
2. When you want to publish your visualization, `import visualization` to the appropriate state.
3. Set up your state.
   - If you are publishing a regular marker, then set `self._markers = kwargs['markers']`.
   - If you are publishing an interactive marker, then set `self._server = kwargs['interactive_marker_server']`.
3. Call your visualization code, passing in the marker publisher or the interactive marker server.
   ```
   visualization.publish_base(self._markers, target_in_shelf_frame)
   visualization.publish_gripper(self._im_server, pose_target, 'grasp_target')
   ```
4. If your visualization requires a new display to be added, then set your rviz config to some sane defaults, and commit your rviz config to `pr2_pick/config`.