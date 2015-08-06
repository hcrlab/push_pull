import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point, PointStamped
import time
import moveit_commander
from moveit_msgs.msg import Grasp, PlanningScene, PlanningSceneComponents 
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
	GetPositionIKRequest, GetPlanningScene
import os
from pr2_pick_main import handle_service_exceptions
import rospkg
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import smach
from std_msgs.msg import Header, String
import tf
import outcomes
from pr2_pick_manipulation.srv import GetPose, MoveArm, SetGrippers, MoveArmIkRequest
from pr2_pick_perception.msg import Box, Cluster2, BoundingBox
from pr2_pick_contest.msg import Record
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest, PlanarPrincipalComponentsRequest, \
	DeleteStaticTransformRequest, BoxPointsResponse #, Cluster
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
import numpy as np
import visualization as viz
import rosbag
from visualization_msgs.msg import Marker

class GraspPlanner(smach.State):
	''' Grasps an item in the bin. '''
	name = 'GRASPPLANNER'

	def __init__(self, **services):
		smach.State.__init__(
			self,
			outcomes=[
				outcomes.GRASP_PLAN_SUCCESS,
				outcomes.GRASP_PLAN_FAILURE,
				outcomes.GRASP_PLAN_NONE
			],
			input_keys=['bin_id', 'debug', 'target_cluster', 'current_target',
						'item_model', 'target_descriptor', 're_grasp_attempt'],
			output_keys=['re_sense_attempt', 'previous_item', 'bounding_box_pose']
		)

		self._set_grippers = services['set_grippers']
		self._get_grippers = services['get_grippers']
		self._tuck_arms = services['tuck_arms']
		self._moveit_move_arm = services['moveit_move_arm']
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

		self.bag = rosbag.Bag("bagfiles/data.bag" , 'w')
		self.bag_data = Record()

	# Set params for grasp planner
	def call_set_params(self, side_step = 0.02, palm_step = 0.005, overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = True, pregrasp_just_outside_box = True, backoff_depth_steps = 5):
		
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
			
	def call_plan_point_cluster_grasp(self, cluster):
		req = GraspPlanningRequest()
		rospy.loginfo("TESTE frame id: " + cluster.header.frame_id)
		req.target.reference_frame_id = 'base_footprint' 
		req.target.cluster = cluster
		req.arm_name = "left_arm"
		service_name = "plan_point_cluster_grasp"
		rospy.loginfo("waiting for plan_point_cluster_grasp service")
		rospy.wait_for_service(service_name)
		rospy.loginfo("service found")
		serv = rospy.ServiceProxy(service_name, GraspPlanning)
		try:
			res = serv(req)
		except rospy.ServiceException, e:
			rospy.logerr("error when calling plan_point_cluster_grasp: %s"%e)  
			return 0
		if res.error_code.value != 0:
			return []

		return res.grasps


	#call plan_point_cluster_grasp_action to get candidate grasps for a cluster
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
		for i in range(5):
			scene.remove_world_object("table")
			scene.remove_world_object("shelf1")
			scene.remove_world_object("shelf")
			scene.remove_world_object("shelf2")
			scene.remove_world_object("shelf3")

		wall_pose1 = PoseStamped()
		wall_pose1.header.frame_id = "odom_combined"
		wall_pose1.pose.position.x = 0.9
		wall_pose1.pose.position.y = 0.045
		wall_pose1.pose.position.z = 0.94

		wall_pose2 = PoseStamped()
		wall_pose2.header.frame_id = "odom_combined"
		wall_pose2.pose.position.x = 0.9
		wall_pose2.pose.position.y = 0.41
		wall_pose2.pose.position.z = 0.94

		wall_pose3 = PoseStamped()
		wall_pose3.header.frame_id = "odom_combined"
		wall_pose3.pose.position.x = 0.9
		wall_pose3.pose.position.y = 0.20
		wall_pose3.pose.position.z = 1.16

		rate = rospy.Rate(1)
		for i in range(5):
			#scene.add_box("table", table_pose, (0.38, 0.38, 0.78))
			scene.add_box("shelf1", wall_pose1, (0.38, 0.015, 0.38 ))
			scene.add_box("shelf2", wall_pose2, (0.38, 0.015, 0.38 ))
			scene.add_box("shelf3", wall_pose3, (0.38, 0.38, 0.015 ))
			rospy.sleep(1)
			rate.sleep()

	def save_image(self, image):
		self.bag_data.image = image

	@handle_service_exceptions(outcomes.GRASP_FAILURE)
	def execute(self, userdata):

		rospy.loginfo("Starting Grasp Planner")

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
		bin_ids = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
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
		#for i in range(10):
		#	rospy.loginfo("Removing Planning Scene")
                #        scene.remove_world_object("table")
                #        scene.remove_world_object("shelf1")
                #        scene.remove_world_object("shelf")
                #        scene.remove_world_object("shelf2")
                #        scene.remove_world_object("shelf3")

		# Convert cluster PointCloud2 to PointCloud
		rospy.loginfo("Waiting for convert_pcl service")
		self.convert_pcl.wait_for_service()
		rospy.loginfo("PCL service found")
		self._cluster2 = Cluster2()
		self._cluster2.pointcloud = self.convert_pcl(userdata.target_cluster.pointcloud).pointcloud      

		# Save pointcloud
		self.bag_data.pointcloud2 = self._cluster2.pointcloud
		#self.bag.write('/head_mount_kinect/depth/points', self.bag_data.pointcloud2 )


		self._cluster = userdata.target_cluster
		self._cluster2.header = userdata.target_cluster.header
		self._cluster2.pointcloud.header = userdata.target_cluster.header
		self._cluster2.id = userdata.target_cluster.id
		self.bag_data.pointcloud2 = self._cluster.pointcloud
		tf_broadcaster = tf.TransformBroadcaster()
		tf_listener = tf.TransformListener()

		# Set params for grasp planner
		self.call_set_params(overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = False, pregrasp_just_outside_box = True, backoff_depth_steps = 1)

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
		#self.bag.write('pr2_pick_visualization', self.bag_data.marker_boundingbox )
		#self.bag.write('pr2_pick_perception/BoundingBox', self.bag_data.boundingbox )
		
		# Adding bouding box to the scene
		#rospy.loginfo("Adding bounding box to the scene")
		#for i in range(10):
		#    scene.add_box("bbox", box_pose, 
		#    (box_dims.x - 0.01, 
		#    box_dims.y - 0.01, 
		#    box_dims.z - 0.01))
		#    rospy.sleep(0.1)

		# Plan Grasp
		grasps = self.call_plan_point_cluster_grasp_action(self._cluster2.pointcloud,self._cluster.header.frame_id )
		rospy.loginfo("Number of grasps: ")
		rospy.loginfo(len(grasps))

		grasp_poses = [grasp.grasp_pose for grasp in grasps]
		grasp_not_stamped = []
		for pose_stamped in grasp_poses:
			grasp_not_stamped.append(pose_stamped.pose)

		# Grasp planner found possible grasps
		if(len(grasps) > 0):

			# Hard code pre grasp state
			pre_grasp_pose = PoseStamped()
			pre_grasp_pose.header.frame_id = "bin_K"
			pre_grasp_pose.pose.position.x = -0.30
			pre_grasp_pose.pose.position.y = 0.0
			pre_grasp_pose.pose.position.z = 0.20
			pre_grasp_pose.pose.orientation.x = 1.0
			pre_grasp_pose.pose.orientation.y = 0.0
			pre_grasp_pose.pose.orientation.z = 0.0
			pre_grasp_pose.pose.orientation.w = 0.0

			# Go to pre grasp
			for i in range (5): 
				viz.publish_gripper(self._im_server, pre_grasp_pose , 'grasp_target') 
			if self._debug:
			   raw_input('(Debug) Press enter to continue >')
	   
			
			success_pre_grasp = self._moveit_move_arm(pre_grasp_pose, 
													0.005, 0.005, 12, 'left_arm',
													False).success
			grasp_poses = sorted(grasp_poses, key=lambda grasp: grasp.pose.position.x)
			print(grasp_poses) 
			# Analyze and perform grasps
			for grasp in grasp_poses:

				# Visualize the gripper in the grasp position
				rospy.loginfo("\n\nPossible Grasp: \n")
				rospy.loginfo(grasp)
				for i in range(10):		
					viz.publish_gripper(self._im_server, grasp, 'grasp_target')

				# Test if grasp is going to hit the shelf
				success_grasp = self._moveit_move_arm(grasp,
													0.005, 0.005, 12, 'left_arm',
													True).success
			
				if(success_grasp == True):
					rospy.loginfo("The object is graspable.")
					self._tts.publish("The object is graspable.")
					time.sleep(2) 
					
					grasp_object = raw_input("Do you want to grasp the object? (y)es or (n)o")

					if(grasp_object == 'y' or grasp_object == 'yes'):

						# Visualize the gripper in the grasp position
						viz.publish_gripper(self._im_server, grasp, 'grasp_target')

						self._tts.publish("Grasping object.")
						rospy.loginfo("Grasping object.")

						# Grasp the object
						possible_grasp = self._moveit_move_arm(grasp,
														0.005, 0.005, 12, 'left_arm',
														False).success
						
						if(possible_grasp == True):
							rospy.loginfo("Good grasp!")
								
							# Close gripper to grasp object
							rospy.loginfo('Close Hand')
							if self._debug:
								raw_input('(Debug) Press enter to continue >')
							self._set_grippers.wait_for_service()
							grippers_open = self._set_grippers(open_left=False, open_right=False, effort=userdata.item_model.grasp_effort)
							gripper_states = self._get_grippers()
							if not gripper_states.left_open:
								self._set_grippers(open_left=False, open_right=False, effort=-1)
							self.bag_data.is_graspable = True
							self.bag.write('record', self.bag_data)
							self.bag.close()
							return outcomes.GRASP_PLAN_SUCCESS

						else:

							self.bag_data.is_graspable = False
							self.bag.write('record',self.bag_data)
							self.bag.close()
							self.bag_data.is_graspable = False
							self.bag.write('record', bag_data)
							self.bag.close()

							rospy.loginfo("It was not possible to grasp the object.")


		# No grasps found
		rospy.loginfo("The object is not graspable.")
		self._tts.publish("The object is not graspable.")
		time.sleep(2)
		self.bag_data.is_graspable = False
		self.bag.write('record', self.bag_data)
		self.bag.close()
		return outcomes.GRASP_PLAN_NONE

