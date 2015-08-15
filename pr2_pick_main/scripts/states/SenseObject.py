from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception import DataSaver
from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import SegmentItemsRequest
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import outcomes
import rospy
import smach
from pr2_pick_manipulation.srv import MoveHead
import visualization as viz
import rospkg
import rosbag
from pr2_pick_contest.msg import Record
from pr2_pick_perception.srv import DeleteStaticTransformRequest
from pr2_pick_perception.msg import Cluster2, BoundingBox
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class SenseObject(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_OBJECT'

    def __init__(self, **services):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.SENSE_OBJECT_BEFORE_SUCCESS,
                outcomes.SENSE_OBJECT_AFTER_SUCCESS,
                outcomes.SENSE_OBJECT_FAILURE],
            input_keys=['debug', 'trial_number', 'is_before'],
            output_keys=['bounding_box'])

        self._segment_items = services['segment_items']
        self._move_head = services['move_head']
        self._crop_shelf = services['crop_shelf']
        self._tts = services['tts']
        self._im_server = services['interactive_marker_server']
        self._delete_static_tf = services['delete_static_tf']
        self._markers = services['markers']
        self.convert_pcl = services['convert_pcl_service']


    #call find_cluster_bounding_box to get the bounding box for a cluster
    def call_find_cluster_bounding_box(self, cluster):
        req = FindClusterBoundingBoxRequest()
        req.cluster = cluster
        service_name = "find_cluster_bounding_box"
        rospy.loginfo("waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
        try:
            res = serv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)  
            return 0
        if not res.error_code:
            return (res.pose, res.box_dims)
        else:
            return (None, None)

    def save_image(self, image):
        self.bag_data.image = image


    @handle_service_exceptions(outcomes.SENSE_OBJECT_FAILURE)
    def execute(self, userdata):

        # Find object cluster
        self._move_head.wait_for_service()
        move_head_success = self._move_head(0, 0, 0, 'bin_K')
      
        if userdata.is_before:
            ########
            rospy.loginfo('Please prepare object and press ready.')
            self._tts.publish('Please prepare object and press ready.')
            raw_input("Press enter after placing the item.")
            ########
        else:
            rospy.loginfo('Sensing object after tool action.')
            self._tts.publish('Sensing object after tool action.')            

        # Crop shelf.
        crop_request = CropShelfRequest(cellID='K')
        self._crop_shelf.wait_for_service()
        crop_response = self._crop_shelf(crop_request)

        # Segment items
        segment_request = SegmentItemsRequest(cloud=crop_response.cloud, items="crayola_64_ct")
        self._segment_items.wait_for_service()
        segment_response = self._segment_items(segment_request)
        clusters = segment_response.clusters.clusters
        rospy.loginfo('[SenseBin] Found {} clusters.'.format(len(clusters)))
        if len(clusters) == 0:
            rospy.logerr('[SenseBin]: No clusters found!')
            return outcomes.SENSE_OBJECT_FAILURE
        elif len(clusters) > 1:
            rospy.logwarn('[SenseBin]: There are more than 1 clusters! Will use cluster 0.')
        target_cluster = clusters[0]

        # Visualize segments
        cluster_markers = []
        for i, cluster in enumerate(clusters):
            # Publish visualization
            points = pc2.read_points(cluster.pointcloud, skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
            if len(point_list) == 0:
                rospy.logwarn('[SenseBin]: Cluster with 0 points returned!')
                continue
            m = viz.publish_cluster(self._markers, point_list,
                                'bin_K', 'bin_K_items', i)
            cluster_markers.append(m)


        # CREATE BEFORE BAG FILE 
        rospack = rospkg.RosPack()
        path = rospack.get_path('pr2_pick_main') + '/data/exploration/'
        
        if userdata.is_before:
            filename = path + 'trial' + str(userdata.trial_number) + '_before.bag'
        else:
            filename = path + 'trial' + str(userdata.trial_number) + '_after.bag'

        self.bag = rosbag.Bag(filename, 'w')
        self.bag_data = Record()
        rospy.loginfo("Opened bag file to save information before the tool action.")
        rospy.Subscriber("/head_mount_kinect/rgb/image_color", Image, self.save_image)
        self.bag_data.marker_pointcloud = cluster_markers[0]

        # Delete any leftover transforms from previous runs
        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "bin_K"
        req.child_frame = "object_axis"
        self._delete_static_tf(req)
        req = DeleteStaticTransformRequest()
        req.child_frame = "bin_K"
        req.parent_frame = "object_axis"
        self._delete_static_tf(req)
        req = DeleteStaticTransformRequest()
        req.child_frame = "bin_K"
        req.parent_frame = "bounding_box"
        self._delete_static_tf(req)

        # Set variables
        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "base_footprint"
        req.child_frame = "head_yaw"
        self._delete_static_tf(req)

        # Convert cluster PointCloud2 to PointCloud and save
        rospy.loginfo("Waiting for convert_pcl service")
        self.convert_pcl.wait_for_service()
        rospy.loginfo("PCL service found")
        target_cluster2 = Cluster2()
        target_cluster2.pointcloud = self.convert_pcl(target_cluster.pointcloud).pointcloud      
        self.bag_data.pointcloud2 = target_cluster2.pointcloud

        target_cluster2.header = target_cluster.header
        target_cluster2.pointcloud.header = target_cluster.header
        target_cluster2.id = target_cluster.id
        self.bag_data.pointcloud2 = target_cluster.pointcloud

        # Get the bounding box
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(target_cluster2.pointcloud)
        box_pose.header.frame_id = target_cluster.header.frame_id
        bounding_box = BoundingBox()
        bounding_box.pose = box_pose
        bounding_box.dimensions = box_dims
        # Publish Bounding Box
        marker_bounding_box = viz.publish_bounding_box(self._markers, box_pose, 
                     (box_dims.x), 
                     (box_dims.y), 
                     (box_dims.z),
                     1.0, 0.0, 0.0, 0.5, 1)
        # Saving bounding box
        self.bag_data.boundingbox = bounding_box
        self.bag_data.marker_boundingbox = marker_bounding_box
        userdata.bounding_box = bounding_box

        self.bag_data.is_graspable = False
        self.bag.write('record', self.bag_data)
        self.bag.close()
	

        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')

        userdata.is_before = not userdata.is_before
        if userdata.is_before:
            userdata.trial_number += 1
            return outcomes.SENSE_OBJECT_AFTER_SUCCESS
        else:
            return outcomes.SENSE_OBJECT_BEFORE_SUCCESS
