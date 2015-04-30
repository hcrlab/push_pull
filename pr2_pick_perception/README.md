# Perception
Contains code for the perception side of the Amazon picking challenge.

## Services
### Shelf localization
Localizes the shelf by fitting it to a shelf model.
The coordinate frame for the shelf is at the bottom center.
```bash
roslaunch pr2_pick_perception perception.launch
rosservice call /perception/localize_shelf
```

### Item clustering
Gets the clusters in a particular bin, given the full point cloud, and assuming that the shelf and bin TFs are being published.

### Item descriptors
Given a cluster, returns a descriptor of it.

### Planar PCA
Gets the 2 PCA components of a cluster on the XY plane.

Example usage in the state machine:
```py
self._get_planar_pca = kwargs['get_planar_pca']
self._get_planar_pca.wait_for_service()
response = self._get_planar_pca(cluster) # cluster is a pr2_pick_perception/Cluster
# response.first_component is the eigenvector with the larger eigenvalue. It is a quaternion relative to the point cloud frame.
# response.second_component is the eigenvector with the smaller eigenvalue.
# response.eigenvalue1 is the larger eigenvalue.
# response.eigenvalue2 is the smaller eigenvalue.
```

### Find Centroid
Returns the centroid of a point cloud.

```py
from pr2_pick_perception.srv import FindCentroid
find_centroid = rospy.ServiceProxy('perception/find_centroid', FindCentroid)
find_centroid.wait_for_service()
# cluster is a pr2_pick_perception/Cluster
point_stamped = find_centroid(cluster)
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

### Running unit tests.
`catkin_make run_tests`, or if using [catkin_tools](http://catkin-tools.readthedocs.org/en/latest/index.html):
`catkin build --this --verbose --catkin-make-args run_tests`

### Debugging
You need to remove `shelf_recognition_KinPR2.launch` from `perception.launch`.
`perception.launch` needs to be run on the robot, because it splits the Kinect work across the robot's computers.
`roslaunch shelf_recognition_KinPR2.launch debug:=true` needs to be run on your desktop, because the visualization doesn't work over `ssh -X`.

This will bring up several visualizations in a row.
1. Raw data. Press `r` to fix the reference frame, `q` to continue.
2. Cropped and downsampled point cloud. Press `r` to fix the reference frame, `q` to continue. This is a good time to check if parts of the shelf have been erroneously cropped.
3. Segmented point cloud. Press `r` to fix the reference frame, `q` to continue.
4. The matched model overlaid with the raw data.

## perception.launch
This contains a launch file called perception.launch, which starts the Kinect, the tilt laser, and all services defined in this package.

## Bag file recording
There are scripts for recording various topics in the scripts folder.

```bash
roslaunch pr2_pick_perception perception.launch
./record_kinect.sh # Records 2 seconds of Kinect data.
```