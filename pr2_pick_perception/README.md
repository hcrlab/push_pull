# Perception
Contains code for the perception side of the Amazon picking challenge.

## perception.launch
This contains a launch file called perception.launch, which starts the Kinect, the fingertip sensor, and the shelf localization service.

## Services
### Shelf localization
```
roslaunch pr2_pick_perception perception.launch
rosservice call /perception/localize_shelf
```

### Static transform publisher
There's a service for broadcasting static transforms.
Use `SetStaticTransform` to add or update a static transform.
Use `DeleteStaticTransform` to delete a static transform.

Sample code:
```py
from pr2_pick_perception.srv import SetStaticTransform

transform = TransformStamped()
transform.header.frame_id = 'odom_combined' # Parent ("from this frame") frame. Make sure it's a fixed frame.
transform.header.stamp = rospy.Time.now()
transform.transform.translation = ...
transform.transform.rotation = ...
transform.child_frame_id = 'shelf'

set_static_tf = rospy.ServiceProxy('perception/set_static_transform', SetStaticTransform)
set_static_tf.wait_for_service()
set_static_tf(transform)
```

#### Debugging
You need to remove `shelf_recognition_KinPR2.launch` from `perception.launch`.
`perception.launch` needs to be run on the robot, because it splits the Kinect work across the robot's computers.
`roslaunch shelf_recognition_KinPR2.launch debug:=true` needs to be run on your desktop, because the visualization doesn't work over `ssh -X`.

This will bring up several visualizations in a row.
1. Raw data. Press `r` to fix the reference frame, `q` to continue.
2. Cropped and downsampled point cloud. Press `r` to fix the reference frame, `q` to continue. This is a good time to check if parts of the shelf have been erroneously cropped.
3. Segmented point cloud. Press `r` to fix the reference frame, `q` to continue.
4. The matched model overlaid with the raw data.

## Bag file recording
There are scripts for recording various topics in the scripts folder.

```bash
roslaunch pr2_pick_perception perception.launch
./record_kinect.sh # Records 2 seconds of Kinect data.
```

## Libraries
### mock_perception.h
A mock perception library.
You explicitly specify where the items are in each bin.
Later, you can get the items back out.

## Executables
### shelf_localization.cpp
The code for the shelf localization service.

### mock_perception_easy_boxes.cpp
An action server for `GetItems` for the travel support demo.
It just tells the robot that the next object is always at more or less the same pose relative to the robot (since the robot drives to each bin).