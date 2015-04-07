# pr2_pick_manipulation

Contains code for PR2 manipulation and robot control.

For documentation, see .h files.

## Services

Use manipulation.launch to launch all these services.

### DriveAngular
Rotates the robot. It's recommended to only use the constants `RIGHT_90`, `LEFT_90`, and `TURN_180` for the 2nd parameter.
```py
from pr2_pick_manipulation.srv import DriveAngular, DriveAngularRequest
rospy.wait_for_service('drive_angular_service')
drive_angular = rospy.ServiceProxy('drive_angular_service', DriveAngular)
# Turn right 90 degrees at 0.1 radians / second
drive_angular(0.1, DriveAngularRequest.RIGHT_90)
```

### DriveLinear
Drives the robot in a straight line.

```py
from pr2_pick_manipulation.srv import DriveLinear
rospy.wait_for_service('drive_linear_service')
drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
drive_linear(0, 0.1, 0.2) # Drive left at 0.1 meters / second for 20 cm.
```

### MoveTorso
Sets the robot's torso height.
```py
from pr2_pick_manipulation.srv import MoveTorso, MoveTorsoRequest
rospy.wait_for_service('torso_service')
move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
move_torso(0.1) # Move torso to position 10 cm up.
move_torso(MoveTorsoRequest.MAX_HEIGHT) # Move torso to maximum height.
```

### SetGrippers
Opens or closes the grippers.
```py
from pr2_pick_manipulation.srv import SetGrippers
rospy.wait_for_service('gripper_service')
set_grippers = rospy.ServiceProxy('gripper_service', SetGrippers)
set_grippers(False, True) # Close left hand and open right hand.
```

### TuckArms
Tucks or untucks the arms.
```py
from pr2_pick_manipulation.srv import TuckArms
rospy.wait_for_service('tuck_arms_service')
tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
tuck_arms(False, True) # Tuck the left arm and untuck the right arm.
```

## Libraries
### gripper.h
C++ action client wrapper for opening and closing the gripper.

Sample usage:
```cpp
Gripper right_gripper("r_gripper_controller/gripper_action");
right_gripper.Open();
right_gripper.Close();
```

### torso.h
C++ action client wrapper for moving the torso up and down

Sample usage:
```cpp
Torso torso;
torso.SetHeight(0.1);
torso.SetHeight(Torso::kMaxHeight);
torso.SetHeight(Torso::kMinHeight);
```

### driver.h
Library for driving the robot using the low-level base controller and odometry.

Sample usage:
```cpp
pr2_pick_manipulation::RobotDriver driver();

RobotDriver driver;
// Drive left for 0.25 meters at 0.125 m/s
driver.DriveLinear(0, 0.125, 0.25);
// Rotate clockwise 90 degrees.
driver.DriveAngular(-0.5, M_PI/2);
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