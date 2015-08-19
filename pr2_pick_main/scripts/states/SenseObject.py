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
from pr2_pick_contest.msg import Record, Trial, TrialParams 
from pr2_pick_perception.srv import DeleteStaticTransformRequest
from pr2_pick_perception.msg import Cluster2, BoundingBox
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from pr2_pick_main.web_interface import WebInterface
from std_msgs.msg import String, Int32


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
            input_keys=['debug', 'is_explore', 'is_before', 'current_trial_num',
            'current_trial', 'before_record', 'action_params'],
            output_keys=['bounding_box', 'is_before', 'before_record'])

        self._segment_items = services['segment_items']
        self._move_head = services['move_head']
        self._crop_shelf = services['crop_shelf']
        self._tts = services['tts']
        self._im_server = services['interactive_marker_server']
        self._delete_static_tf = services['delete_static_tf']
        self._markers = services['markers']
        self.convert_pcl = services['convert_pcl_service']

        self._interface = WebInterface()
        self._positions = ["Position 1: Front Centre",
        "Position 2: Front Left",
        "Position 3: Front Right",
        "Position 4: Back"]
        self._orientations = ["Orientation 1: Facing Side",
        "Orientation 2: Facing Front",
        "Orientation 3: Angled"]


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

    def log_message(self, message):
        rospy.loginfo(message)
        self._tts.publish(message)
        self._interface.display_message(message)

    @handle_service_exceptions(outcomes.SENSE_OBJECT_FAILURE)
    def execute(self, userdata):

        # Find object cluster
        self._move_head.wait_for_service()
        move_head_success = self._move_head(0, 0, 0, 'bin_K')
      
        if userdata.is_before:
            if not userdata.is_explore:
                self.log_message('Starting trial ' + str(userdata.current_trial_num) 
                    + '. Please prepare object and press Ready.')
                item_name = userdata.current_trial["item_name"]
                position = userdata.current_trial["position"]
                orientation = userdata.current_trial["orientation"]
                message = (
                    "Please prepare following configuration\n" +
                    "_______________________________\n\n" +
                    "Item: " + str(item_name) + "\n" +
                    str(self._positions[position]) + "\n" +
                    str(self._orientations[orientation]) + "\n" +
                    "_______________________________\n" +
                    "Then press Ready")
                self._interface.ask_choice(message, ['Ready'])
            else:
                self.log_message('Starting new trial.' +
                    '. Please prepare object and press Ready.')
                self._interface.ask_choice(
                    'Please prepare object and press Ready.',
                    ['Ready'])
            
            self.log_message('Sensing object before tool action.')
        else:
            self.log_message('Sensing object after tool action.')

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
            self.log_message('Failed to sense object.')
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

        if userdata.is_before:
            userdata.before_record = self.bag_data
            #filename = path + 'trial' + str(userdata.current_trial_num) + '_before.bag'
        else:
            if not userdata.is_explore:
                rospack = rospkg.RosPack()
                item_name = userdata.current_trial["item_name"]
                orientation = userdata.current_trial["orientation"]
                position = userdata.current_trial["position"]
                action = userdata.current_trial["action"]

                path = rospack.get_path('pr2_pick_main') + '/data/experiments/'
                bag_file_name = ("TRIAL_" + str(userdata.current_trial_num) + "_" +
                    str(item_name) +
                    "_position_" + str(position) +
                    "_orientation_" + str(orientation) +
                    "_action_" + str(action) + ".bag")

                trial_params = TrialParams()
                trial_params.item_name = String(item_name)
                trial_params.orientation = Int32(orientation)
                trial_params.position = Int32(position)
                trial_params.action = String(action)
                trial_params.action_params = userdata.action_params

                bag = rosbag.Bag(bag_file_path + bag_file_name , 'w')
                trial.before = userdata.before_record
                trial.after = self.after_record
                trial.params = trial_params
                bag.write('trial', trial)
                bag.close()

            userdata.before_record = None

        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')

        self.log_message('Sensing complete.')

        if userdata.is_before:
            userdata.is_before = False
            return outcomes.SENSE_OBJECT_BEFORE_SUCCESS
        else:
            userdata.is_before = True
            self.log_message('Trial ' + str(userdata.current_trial_num) + ' complete.')
            rospy.sleep(2)
            return outcomes.SENSE_OBJECT_AFTER_SUCCESS
