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
