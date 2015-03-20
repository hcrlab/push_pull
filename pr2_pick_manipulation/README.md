# pr2_pick_manipulation

Contains code for PR2 manipulation and robot control.

For documentation, see .h files.

## Libraries
### gripper.h
C++ action client wrapper for opening and closing the gripper.

Sample usage:
```cpp
Gripper right_gripper("r_gripper_controller/gripper_action");
right_gripper.open();
right_gripper.close();
```

### driver.h
Library for driving the robot using the low-level base controller and odometry.

Sample usage:
```cpp
ros::NodeHandle node_handle;
ros::Publisher cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>(
  "base_controller/command", 1);
pr2_pick_manipulation::RobotDriver driver(cmd_vel_pub);

// Specify the speed in the x and y direction in meters/second as well as
// the angular velocity in radians/second.
geometry_msgs::Twist base_cmd;
base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
base_cmd.linear.y = 0.125;

// Drive until the robot has been displaced 0.25 meters from its starting
// position.
driver.Drive(base_cmd, 0.25);
```

### arm_navigator.h
Interface and implementations for moving the arm using motion planning.

Sample usage:
```cpp
MoveGroup group("right_arm");
ArmNavigatorInterface* right_arm = new MoveItArmNavigator(group);
right_arm.MoveToPoseGoal(pose, false);
```

## End-effector reader tool
A tool for easily getting the transform for one of the arms relative to the robot.
Currently, the transform is hard-coded to be `r_wrist_roll_link` relative to `base_footprint`.

Usage:

```bash
roslaunch pr2_pick_manipulation ee_reader.launch

# Serve index.html on port 8080 so you can see the result on your phone or laptop.
roscd pr2_pick_manipulation/www
python -m SimpleHTTPServer 8080 .

# Disable the right arm
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
```

Now you should be able to navigate to `{your hostname}.cs.washington.edu:8080` on your phone or laptop and see tf information for the right arm.
As you move the arm around, the tf information should change.
Press 'Pause' to save a pose that you want to copy down.
Press 'Start' again to starting listening to tf again.

If you want the left arm or a different transform altogether, modify index.html

When you're done, you should restart the arm controller:
```bash
rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller
```