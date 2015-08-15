from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception import DataSaver
from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import SegmentItemsRequest
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import outcomes
import rospy
import smach
from pr2_pick_manipulation.srv import MoveHead
import visualization as viz


class SenseObjectBefore(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_OBJECT_BEFORE'

    def __init__(self, **kwargs):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.SENSE_BIN_SUCCESS, outcomes.SENSE_BIN_NO_OBJECTS,
                      outcomes.SENSE_BIN_FAILURE],
            input_keys=['bin_id', 'debug', 'current_target',
                        'current_bin_items', 're_sense_attempt', 'previous_item'],
            output_keys=['clusters', 'target_cluster', 'target_descriptor',
                         'target_model', 're_grasp_attempt', 'current_target'])

        self._segment_items = kwargs['segment_items']
        self._get_item_descriptor = kwargs['get_item_descriptor']
        self._classify_target_item = kwargs['classify_target_item']
        self._lookup_item = kwargs['lookup_item']

        self._move_head = services['move_head']
        self._crop_shelf = services['crop_shelf']
        self._tts = services['tts']
        self._tf_listener = tf.TransformListener() 
        self._im_server = services['interactive_marker_server']
        self._set_static_tf = services['set_static_tf']
        self._delete_static_tf = services['delete_static_tf']
        self._markers = services['markers']
        self._move_arm_ik = services['move_arm_ik']
        self._fk_client = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self._ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._lookup_item = services['lookup_item']
        self._get_planar_pca = services['get_planar_pca']
        self.convert_pcl = services['convert_pcl_service']
        self._wait_for_transform_duration = rospy.Duration(5.0)
        self._cluster = None
        self._debug = False
        self.debug_grasp_pub = rospy.Publisher('debug_grasp_pub', String, queue_size=10)

        #name_file = raw_input("Name of the bag file: ")
        #self.bag = rosbag.Bag("bagfiles/" + name_file , 'w')
        #self.bag_data = Record()

    # Set params for grasp planner
    def call_set_params(self, side_step = 0.02, palm_step = 0.005, 
        overhead_grasps_only = False, side_grasps_only = False,
        include_high_point_grasps = True, pregrasp_just_outside_box = True,
        backoff_depth_steps = 5):
        
        req = SetPointClusterGraspParamsRequest()
        req.height_good_for_side_grasps = 0.02
        req.gripper_opening = 0.083
        req.side_step = 0.01
        req.palm_step = palm_step
        req.overhead_grasps_only = overhead_grasps_only
        req.side_grasps_only = side_grasps_only
        req.include_high_point_grasps = include_high_point_grasps
        req.pregrasp_just_outside_box = pregrasp_just_outside_box
        req.backoff_depth_steps = backoff_depth_steps
        req.randomize_grasps = False
        rospy.loginfo("waiting for set params service")
        rospy.wait_for_service("set_point_cluster_grasp_params")
        serv = rospy.ServiceProxy("set_point_cluster_grasp_params", SetPointClusterGraspParams)
        try:
            res = serv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling set params: %s"%e)
            return 0
        return 1

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

    def call_plan_point_cluster_grasp_action(self, cluster, frame_id):
        goal = GraspPlanningGoal()
        goal.target.reference_frame_id = frame_id
        goal.target.cluster = cluster
        goal.arm_name = "left_arm"
        action_name = "plan_point_cluster_grasp"
        rospy.loginfo("waiting for plan_point_cluster_grasp action")
        client = actionlib.SimpleActionClient(action_name, GraspPlanningAction)
        client.wait_for_server()
        rospy.loginfo("action found")
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        if not result or result.error_code.value != 0:
            return []
        return result.grasps
    
    def add_shelf_to_scene(self, scene):
        for i in range(10):
            scene.remove_world_object("bbox")
            scene.remove_world_object("shelf1")
            scene.remove_world_object("shelf")
            scene.remove_world_object("shelf2")
            scene.remove_world_object("shelf3")

        wall_pose1 = PoseStamped()
        wall_pose1.header.frame_id = "odom_combined"
        wall_pose1.pose.position.x = 0.9
        wall_pose1.pose.position.y = 0.025
        wall_pose1.pose.position.z = 0.94

        wall_pose2 = PoseStamped()
        wall_pose2.header.frame_id = "odom_combined"
        wall_pose2.pose.position.x = 0.9
        wall_pose2.pose.position.y = 0.385
        wall_pose2.pose.position.z = 0.94

        wall_pose3 = PoseStamped()
        wall_pose3.header.frame_id = "odom_combined"
        wall_pose3.pose.position.x = 0.9
        wall_pose3.pose.position.y = 0.20
        wall_pose3.pose.position.z = 1.14
        
        wall_pose4 = PoseStamped()
                wall_pose4.header.frame_id = "odom_combined"
                wall_pose4.pose.position.x = 0.9
                wall_pose4.pose.position.y = 0.20
                wall_pose4.pose.position.z = 0.76

        rate = rospy.Rate(1)
        for i in range(5):
            #scene.add_box("table", table_pose, (0.38, 0.38, 0.78))
            scene.add_box("shelf1", wall_pose1, (0.38, 0.015, 0.38 ))
            scene.add_box("shelf2", wall_pose2, (0.38, 0.015, 0.38 ))
            scene.add_box("shelf3", wall_pose3, (0.38, 0.38, 0.015 ))
            scene.add_box("shelf4", wall_pose4, (0.38,0.38,0.015))
            rospy.sleep(1)
            rate.sleep()

    @handle_service_exceptions(outcomes.SENSE_BIN_FAILURE)
    def execute(self, userdata):

        if 're_sense_attempt' in userdata and userdata.re_sense_attempt:
            userdata.re_grasp_attempt = True
        else:
            userdata.re_grasp_attempt = False

        rospy.loginfo('Sensing shelf.')
        self._tts.publish('Sensing shelf.')
        self._move_head.wait_for_service()
        move_head_success = self._move_head(0, 0, 0, 'bin_K')
        self._lookup_item.wait_for_service()
        lookup_response = self._lookup_item(item=userdata.current_target)
        target_model = lookup_response.model
        userdata.target_model = target_model
      
        rospy.loginfo('Please prepare object and press ready.')
        self._tts.publish('Please prepare object and press ready.')

        ########
        raw_input("Press enter after placing the item.")
        ########

        current_bin_items = userdata.current_target

        # Crop shelf.
        crop_request = CropShelfRequest(cellID=userdata.bin_id)
        self._crop_shelf.wait_for_service()
        crop_response = self._crop_shelf(crop_request)

        # Segment items
        segment_request = SegmentItemsRequest(cloud=crop_response.cloud, items=current_bin_items)
        self._segment_items.wait_for_service()
        segment_response = self._segment_items(segment_request)
        clusters = segment_response.clusters.clusters
        userdata.clusters = clusters
        rospy.loginfo('[SenseBin] Found {} clusters.'.format(
            len(clusters)))
        if len(clusters) == 0:
            rospy.logerr('[SenseBin]: No clusters found!')
            return outcomes.SENSE_BIN_FAILURE

        for i, cluster in enumerate(clusters):
            # Publish visualization
            points = pc2.read_points(cluster.pointcloud, skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
            if len(point_list) == 0:
                rospy.logwarn('[SenseBin]: Cluster with 0 points returned!')
                continue
            viz.publish_cluster(self._markers, point_list,
                                'bin_{}'.format(userdata.bin_id),
                                'bin_{}_items'.format(userdata.bin_id), i)

        # Classify which cluster is the target item.
        if len(clusters) == 0:
            rospy.logerr('[SenseBin]: No descriptors found!')
            return outcomes.SENSE_BIN_FAILURE
        elif len(clusters) > 1:
            rospy.logwarn('[SenseBin]: There are more than 1 clusters! Will use cluster 0.')

        index = 0
        userdata.target_cluster = clusters[index]


        ########## CREATE BEFORE BAG FILE 

        rospack = rospkg.RosPack()
        path = rospack.get_path('pr2_pick_main') + '/data/exploration/'
        filename = path + 'trial' + str(userdata.trial_number) + '.bag'
        self.bag = rosbag.Bag(filename, 'w')
        self.bag_data = Record()

        rospy.loginfo("Opened bag file to save information before the tool action.")

        # Save rgb image
        rospy.Subscriber("/head_mount_kinect/rgb/image_color", Image, self.save_image)

        # Publish cluster
        points = pc2.read_points(userdata.target_cluster.pointcloud, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        marker_cluster = viz.publish_cluster(self._markers, point_list,
                                'bin_K','bin_K_items', 0)

        # Save marker
        self.bag_data.marker_pointcloud = marker_cluster
        #self.bag.write('pr2_pick_visualization', self.bag_data.marker_pointcloud )

        # Delete any leftover transforms from previous runs
        bin_ids = ["K"]
        for bin_id in bin_ids:
            self._delete_static_tf.wait_for_service()
            req = DeleteStaticTransformRequest()
            req.parent_frame = "bin_" + str(bin_id)
            req.child_frame = "object_axis"
            self._delete_static_tf(req)
            req = DeleteStaticTransformRequest()
            req.child_frame = "bin_" + str(bin_id)
            req.parent_frame = "object_axis"
            self._delete_static_tf(req)
            req = DeleteStaticTransformRequest()
            req.child_frame = "bin_" + str(bin_id)
            req.parent_frame = "bounding_box"
            self._delete_static_tf(req)

        # Set variables
        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "base_footprint"
        req.child_frame = "head_yaw"
        self._delete_static_tf(req)
        self.bin_id = userdata.bin_id
        self._debug = userdata.debug
        self.allowed_grasps = userdata.item_model.allowed_grasps
        self.target_descriptor = userdata.target_descriptor
        self.grasp_multiple_heights = userdata.item_model.grasp_multiple_heights
        self.grasp_wide_end = userdata.item_model.grasp_wide_end
        self._cluster = userdata.target_cluster
        userdata.previous_item = userdata.current_target

        # Add shelf to the scene
        rospy.loginfo("Adding shelf to the scene")
        scene = moveit_commander.PlanningSceneInterface()
        self.add_shelf_to_scene(scene)       

        # Convert cluster PointCloud2 to PointCloud
        rospy.loginfo("Waiting for convert_pcl service")
        self.convert_pcl.wait_for_service()
        rospy.loginfo("PCL service found")
        self._cluster2 = Cluster2()
        self._cluster2.pointcloud = self.convert_pcl(userdata.target_cluster.pointcloud).pointcloud      

        # Save pointcloud
        self.bag_data.pointcloud2 = self._cluster2.pointcloud

        self._cluster = userdata.target_cluster
        self._cluster2.header = userdata.target_cluster.header
        self._cluster2.pointcloud.header = userdata.target_cluster.header
        self._cluster2.id = userdata.target_cluster.id
        self.bag_data.pointcloud2 = self._cluster.pointcloud
        tf_broadcaster = tf.TransformBroadcaster()
        tf_listener = tf.TransformListener()

        # Set params for grasp planner
        self.call_set_params(overhead_grasps_only = False, side_grasps_only = False,
            include_high_point_grasps = False, pregrasp_just_outside_box = True,
            backoff_depth_steps = 1)

        # Get the bounding box
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(self._cluster2.pointcloud)
        userdata.bounding_box_pose = box_pose
        box_pose.header.frame_id = self._cluster.header.frame_id
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
        # Adding bouding box to the scene
        #rospy.loginfo("Adding bounding box to the scene")
        #for i in range(10):
        #    scene.add_box("bbox", box_pose, 
        #    (box_dims.x - 0.03, 
        #    box_dims.y - 0.02, 
        #    box_dims.z - 0.03))
        #    rospy.sleep(0.1)

        # Plan Grasp
        grasps = self.call_plan_point_cluster_grasp_action(
            self._cluster2.pointcloud,
            self._cluster.header.frame_id)
        rospy.loginfo("Number of grasps: ")
        rospy.loginfo(len(grasps))

        grasp_poses = [grasp.grasp_pose for grasp in grasps]
        grasp_not_stamped = []
        for pose_stamped in grasp_poses:
            grasp_not_stamped.append(pose_stamped.pose)
        
        self.bag_data.is_graspable = False
        self.bag.write('record', self.bag_data)
        self.bag.close()
	
        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')

        return outcomes.SENSE_BIN_SUCCESS