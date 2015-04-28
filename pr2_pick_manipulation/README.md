# pr2_pick_manipulation

Contains code for PR2 manipulation and robot control.

## Services

Use manipulation.launch to launch all these services.

### MoveArm
Moves the arm to the target pose, with collision checking.
```py
from pr2_pick_manipulation.srv import MoveArm
move_arm = rospy.ServiceProxy('moveit_service', MoveArm)
move_arm.wait_for_service()
pose_target = PoseStamped()
# ... fill out pose_target.
move_arm(pose_target, position_tolerance=0.001, orientation_tolerance=0.01, planning_time=0, 'right_arm')
```

### MoveArmIk
Moves the arm in a straight line to the target pose, without using collision checking.
```py
from pr2_pick_manipulation.srv import MoveArmIk, MoveArmIkRequest
move_arm_ik = rospy.ServiceProxy('move_arm_ik', MoveArmIk)
move_arm_ik.wait_for_service()
pose_target = PoseStamped()
# ... fill out pose_target.
move_arm(pose_target, MoveArmIkRequest.RIGHT)
```

### DriveToPose
Drives the robot to a given PoseStamped.
```py
from geometry_msgs.msg import PoseStamped()
from pr2_pick_manipulation.srv import DriveToPose
drive_to_pose = rospy.ServiceProxy('drive_to_pose', DriveToPose)
drive_to_pose.wait_for_service()
target = PoseStamped()
target.header.frame_id = 'shelf'
target.pose.position.x = -1
target.pose.position.y = 0
target.pose.position.z = 0
target.pose.orientation.w = 1
drive_to_pose(target, linearVelocity=0.1, angularVelocity=0.1)
```

### DriveAngular
Rotates the robot. It's recommended to only use the constants `RIGHT_90`, `LEFT_90`, and `TURN_180` for the 2nd parameter.
```py
from pr2_pick_manipulation.srv import DriveAngular, DriveAngularRequest
drive_angular = rospy.ServiceProxy('drive_angular_service', DriveAngular)
drive_angular.wait_for_service()
# Turn right 90 degrees at 0.1 radians / second
drive_angular(0.1, DriveAngularRequest.RIGHT_90)
```

### DriveLinear
Drives the robot in a straight line.

```py
from pr2_pick_manipulation.srv import DriveLinear
drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
drive_linear.wait_for_service()
drive_linear(0, 0.1, 0.2) # Drive left at 0.1 meters / second for 20 cm.
```

### MoveTorso
Sets the robot's torso height.
```py
from pr2_pick_manipulation.srv import MoveTorso, MoveTorsoRequest
move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
move_torso.wait_for_service()
move_torso(0.1) # Move torso to position 10 cm up.
move_torso(MoveTorsoRequest.MAX_HEIGHT) # Move torso to maximum height.
```

### SetGrippers
Opens or closes the grippers.
```py
from pr2_pick_manipulation.srv import SetGrippers
set_grippers = rospy.ServiceProxy('set_grippers_service', SetGrippers)
set_grippers.wait_for_service()
set_grippers(False, True) # Close left hand and open right hand.
```

### TuckArms
Tucks or untucks the arms.
```py
from pr2_pick_manipulation.srv import TuckArms
tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
tuck_arms.wait_for_service()
tuck_arms(True, False) # Tuck the left arm and untuck the right arm.
```

## manipulation.launch
Launch file that contains most services related to manipulation.
A notable exception is that move_group.launch is not included.
This is because we've found that MoveIt performs much better in terms of speed and number of plans found when it is run on a separate machine from the robot.

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