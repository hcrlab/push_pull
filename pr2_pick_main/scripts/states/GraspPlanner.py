import actionlib

from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from copy import deepcopy
import datetime
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point, PointStamped
from joblib import Parallel, delayed 
import json
import math
import time
import moveit_commander
from moveit_msgs.msg import Grasp, PlanningScene, PlanningSceneComponents 
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetPlanningScene
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal
from moveit_simple_grasps.msg import GraspGeneratorOptions
import multiprocessing
import os
from pr2_pick_main import handle_service_exceptions
import rospkg
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import smach
from std_msgs.msg import Header, String
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
import scipy
import outcomes
from pr2_pick_manipulation.srv import GetPose, MoveArm, SetGrippers, MoveArmIkRequest
from pr2_pick_perception.msg import Box, Cluster2
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest, PlanarPrincipalComponentsRequest, \
    DeleteStaticTransformRequest, BoxPointsResponse #, Cluster

from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
import numpy as np
import visualization as viz

from visualization_msgs.msg import Marker
#from manipulation_msgs.msg import GraspableObject

class GraspPlanner(smach.State):
    ''' Grasps an item in the bin. '''
    name = 'GRASPPLANNER'
    bounding_box_marker_id = 46976
    bounding_box_pose_marker_id = 46977
    # this and this + 1 are the corner ids
    corner_marker_id = 46978


    # How many pre-grasp gripper positions to attempt
    pre_grasp_attempts = 6
    # Separation in meters between attempted pre-grasp positions
    pre_grasp_attempt_separation = 0.01
    # how many grasp gripper positions to attempt
    grasp_attempts = 8

    # desired distance from palm frame to object centroid
    re_grasp_x_distance = 0.37

    pre_grasp_offset = 0.08

    # approximately half hand thickness
    half_gripper_height = 0.03
    # approximate distance from palm frame origin to palm surface
    dist_to_palm = 0.12
    # approximate distance from palm frame origin to fingertip with gripper closed
    dist_to_fingertips = 0.20

    # approx dist between fingers when gripper open
    gripper_palm_width = 0.08

    # approx height of pads of fingertips
    gripper_finger_height = 0.03

    pre_grasp_height = half_gripper_height + 0.02

    # minimum required grasp quality
    min_grasp_quality = 0.3

    max_grasp_quality = 10000

    # minimum number of points from cluster needed inside gripper for grasp
    min_points_in_gripper = 50

    # max number of points in cluster that can intersect with fingers
    max_finger_collision_points = 7

    max_palm_collision_points = 6

    shelf_bottom_height = None
    shelf_height = None
    shelf_wdith = None

    low_object  = False

    top_shelf = False

    grasp_num = 0

    ik_timeout = rospy.Duration(3.0)
    timer = 0.0
    timer1 = 0.0
    timer2 = 0.0

    min_reachable = 2

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

        self._find_centroid = services['find_centroid']
        self._set_grippers = services['set_grippers']
        self._get_grippers = services['get_grippers']
        self._tuck_arms = services['tuck_arms']
        self._moveit_move_arm = services['moveit_move_arm']
        self._tts = services['tts']
        self._tf_listener = tf.TransformListener() #services['tf_listener']
        self._im_server = services['interactive_marker_server']
        self._set_static_tf = services['set_static_tf']
        self._delete_static_tf = services['delete_static_tf']
        self._markers = services['markers']
        self._move_arm_ik = services['move_arm_ik']

        self._fk_client = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self._ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
        #self._get_points_in_box = rospy.ServiceProxy(name='perception/get_points_in_box', 
        #                                                service_class = BoxPoints,
        #                                                persistent = True)
        self._lookup_item = services['lookup_item']
        self._get_planar_pca = services['get_planar_pca']
        self.convert_pcl = services['convert_pcl_service']

        self._wait_for_transform_duration = rospy.Duration(5.0)

        self._cluster = None
        self._debug = False

        self.debug_grasp_pub = rospy.Publisher('debug_grasp_pub', String, queue_size=10)

        # Shelf heights

        self._shelf_bottom_height_a_c = 1.57
        self._shelf_bottom_height_d_f = 1.35
        self._shelf_bottom_height_g_i = 1.10
        self._shelf_bottom_height_j_l = 0.85

        self._shelf_bottom_heights = {
            'A': self._shelf_bottom_height_a_c,
            'B': self._shelf_bottom_height_a_c,
            'C': self._shelf_bottom_height_a_c,
            'D': self._shelf_bottom_height_d_f,
            'E': self._shelf_bottom_height_d_f,
            'F': self._shelf_bottom_height_d_f,
            'G': self._shelf_bottom_height_g_i,
            'H': self._shelf_bottom_height_g_i,
            'I': self._shelf_bottom_height_g_i,
            'J': self._shelf_bottom_height_j_l,
            'K': self._shelf_bottom_height_j_l,
            'L': self._shelf_bottom_height_j_l
        }
        self._shelf_widths = {
            'A': 0.25,
            'B': 0.28,
            'C': 0.25,
            'D': 0.25, 
            'E': 0.28,
            'F': 0.25,
            'G': 0.25, 
            'H': 0.28, 
            'I': 0.25, 
            'J': 0.25, 
            'K': 0.28, 
            'L': 0.25 
        }

        self._shelf_heights = {
            'A': 0.21,
            'B': 0.21,
            'C': 0.21,
            'D': 0.17, 
            'E': 0.17,
            'F': 0.17,
            'G': 0.17, 
            'H': 0.17, 
            'I': 0.17, 
            'J': 0.21, 
            'K': 0.21, 
            'L': 0.21 
        }

        self._bin_bounds_left = {
            'A': 0.12,
            'B': 0.10,
            'C': 0.10,
            'D': 0.12,
            'E': 0.10,
            'F': 0.10,
            'G': 0.12,
            'H': 0.10,
            'I': 0.10,
            'J': 0.12,
            'K': 0.10,
            'L': 0.10
        }
        self._bin_bounds_right = {
            'A': 0.10,
            'B': 0.10,
            'C': 0.12,
            'D': 0.10,
            'E': 0.10,
            'F': 0.12,
            'G': 0.10,
            'H': 0.10,
            'I': 0.12,
            'J': 0.10,
            'K': 0.10,
            'L': 0.12
        }


    def draw_grasps(self, grasps, frame, ns = 'grasps', pause = 0, frame_locked = False):

        marker = Marker()
        marker_pub = rospy.Publisher('grasp_markers', Marker)
        marker.header.frame_id = frame
        rospy.loginfo("Frame: " + frame)
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = frame_locked
        marker_pub.publish(marker)
        for (grasp_num, grasp) in enumerate(grasps):
            if grasp_num == 0:
                marker.scale.x = 0.015
                marker.scale.y = 0.025
                length_fact = 1.5

            else:
                marker.scale.x = 0.01
                marker.scale.y = 0.015
                length_fact = 1.0

            orientation = grasp.orientation
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            mat = tf.transformations.quaternion_matrix(quat)
            start = [grasp.position.x, grasp.position.y, grasp.position.z]
            x_end = list(mat[:,0][0:3]*.05*length_fact + scipy.array(start))    
            y_end = list(mat[:,1][0:3]*.02*length_fact + scipy.array(start))
            marker.id = grasp_num*3
            marker.points = [Point(*start), Point(*x_end)]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_pub.publish(marker)
            marker.id = grasp_num*3+1
            marker.points = [Point(*start), Point(*y_end)]
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_pub.publish(marker)
            marker.id = grasp_num*3+2
            if pause:
                print "press enter to continue"
                raw_input()
        time.sleep(.5)

    def loginfo(self, string):
        #self.debug_grasp_pub.publish(string)
        rospy.loginfo(string)

    # Set params for grasp planner
    def call_set_params(self, side_step = 0.02, palm_step = 0.005, overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = True, pregrasp_just_outside_box = False, backoff_depth_steps = 5):
        
        req = SetPointClusterGraspParamsRequest()
        req.height_good_for_side_grasps = 0.05
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
        req.arm_name = "right_arm"
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
        goal.arm_name = "right_arm"
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

    def get_ik_position(self, pre_grasp):
        fk_request = GetPositionFKRequest()


        fk_request.header.frame_id = "bin_K"
         
	fk_request.fk_link_names.append("r_wrist_roll_link")

	
        fk_request.robot_state.joint_state = pre_grasp

        fk_response = self._fk_client(fk_request)

        return fk_response
    
    def add_shelf_to_scene(self, scene):
        scene.remove_world_object("table")
        scene.remove_world_object("shelf")

        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "/base_link"
        table_pose.pose.position.x = 0.45
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.40

        rate = rospy.Rate(1)
        for i in range(2):
            scene.add_box("table", table_pose, (0.3, 0.4, 0.02))
            rospy.sleep(2)
            rate.sleep()
    
    @handle_service_exceptions(outcomes.GRASP_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Starting Grasp Planner")


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

        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "base_footprint"
        req.child_frame = "head_yaw"
        self._delete_static_tf(req)

        # Set a bunch of member variables
        # These could be way better organised and consistently named! Sorry!
        if userdata.bin_id < 'D':
            self.top_shelf = True
        else:
            self.top_shelf = False

        self.shelf_width = self._shelf_widths[userdata.bin_id]
        self.shelf_height = self._shelf_heights[userdata.bin_id]
        self.bin_id = userdata.bin_id
        self._debug = userdata.debug
        self.allowed_grasps = userdata.item_model.allowed_grasps
        self.target_descriptor = userdata.target_descriptor
        self.bin_bound_left = self._bin_bounds_left[userdata.bin_id] 
        self.bin_bound_right = self._bin_bounds_right[userdata.bin_id]
        self.grasp_multiple_heights = userdata.item_model.grasp_multiple_heights
        self.grasp_wide_end = userdata.item_model.grasp_wide_end
        self._cluster = userdata.target_cluster
        userdata.previous_item = userdata.current_target
	

	    # Add shelf to the scene
        raw_input("Add shelf to the scene")
	    scene = moveit_commander.PlanningSceneInterface()
	    self.add_shelf_to_scene(scene)       
        rospy.loginfo("Shelf added to the scene")

        # Convert cluster PointCloud2 to PointCloud
        rospy.loginfo("Waiting for convert_pcl service")
        self.convert_pcl.wait_for_service()
        rospy.loginfo("PCL service found")
        self._cluster2 = Cluster2()
    	self._cluster2.pointcloud = self.convert_pcl(userdata.target_cluster.pointcloud).pointcloud      

    	self._cluster = userdata.target_cluster
    	self._cluster2.header = userdata.target_cluster.header
    	self._cluster2.pointcloud.header = userdata.target_cluster.header
    	self._cluster2.id = userdata.target_cluster.id

        tf_broadcaster = tf.TransformBroadcaster()
        tf_listener = tf.TransformListener()

        # Set params for grasp planner
        self.call_set_params(overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = False, pregrasp_just_outside_box = True, backoff_depth_steps = 1)

        # Get the bounding box
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(self._cluster2.pointcloud)
        userdata.bounding_box_pose = box_pose
	box_pose.header.frame_id = self._cluster.header.frame_id

        # Publish Bounding Box
        viz.publish_bounding_box(self._markers, box_pose, 
                     (box_dims.x), 
                     (box_dims.y), 
                     (box_dims.z),
                     1.0, 0.0, 0.0, 0.5, 1)

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

    	    #self.draw_grasps(grasp_not_stamped, self._cluster.header.frame_id, pause = 0)
            #self.draw_grasps(grasp_not_stamped, self._cluster.header.frame_id, pause = 0)

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
    	    #viz.publish_gripper(self._im_server, pre_grasp_pose , 'grasp_target') 
            if self._debug:
                raw_input('(Debug) Press enter to continue >')
	    rospy.loginfo("\n\nPossible Grasp: \n")
	    rospy.loginfo(grasp)
            success_pre_grasp = self._moveit_move_arm(pre_grasp_pose, 
                                                            0.005, 0.005, 12, 'right_arm',
                                                            False).success
    	    # Analyze and perform grasps
            for grasp in grasp_poses:

                # Visualize the gripper in the grasp position
        		
			viz.publish_gripper(self._im_server, grasp, 'grasp_target')
			if(grasp.pose.position.x < 0.01):
                		# Test if grasp is going to hit the shelf
    				success_grasp = self._moveit_move_arm(grasp,
                                                            0.005, 0.005, 12, 'right_arm',
                                                            True).success
			else:
				success_grasp = False
    			if(success_grasp == True):
                    		self._tts.publish("The object is graspable.")
                    		time.sleep(2) 
                    		rospy.loginfo("The object is graspable.")
				rospy.loginfo(grasp)
				pre_grasp_pose = PoseStamped()
            			pre_grasp_pose.header.frame_id = "bin_K"
		                pre_grasp_pose.pose.position.x = -0.30
			        pre_grasp_pose.pose.position.y = 0.0
			        pre_grasp_pose.pose.position.z = 0.20
			        pre_grasp_pose.pose.orientation.x = grasp.pose.orientation.x
			        pre_grasp_pose.pose.orientation.y = 0.0
        			pre_grasp_pose.pose.orientation.z = 0.0
			        pre_grasp_pose.pose.orientation.w = 0.0	
                    		viz.publish_gripper(self._im_server, pre_grasp_pose , 'grasp_target')

				if self._debug:
                			raw_input('(Debug) Press enter to continue >')


				rospy.loginfo("Going to pre grasp position")
				success_pre_grasp = self._moveit_move_arm(pre_grasp_pose,
                                                            0.005, 0.005, 12, 'right_arm',
                                                            False).success
				grasp_object = raw_input("Do you want to grasp the object? (y)es or (n)o")

                    		if(grasp_object == 'y' or grasp_object == 'yes'):

                        # Visualize the gripper in the grasp position
                        		viz.publish_gripper(self._im_server, grasp, 'grasp_target')

                        		self._tts.publish("Grasping object.")

                        	# Try to grasp the object
                        		possible_grasp = self._moveit_move_arm(grasp,
                                                            0.005, 0.005, 12, 'right_arm',
                                                            False).success
                        
                        		if(possible_grasp == True):
                            			rospy.loginfo("Good grasp!")
                            	
                            			# Close gripper to grasp object
                            			self.loginfo('Close Hand')
                           	 		if self._debug:
                                			raw_input('(Debug) Press enter to continue >')
                            			self._set_grippers.wait_for_service()
                            			grippers_open = self._set_grippers(open_left=False, open_right=False,
                                                               effort=userdata.item_model.grasp_effort)
                            			gripper_states = self._get_grippers()
                            			if not gripper_states.right_open:
                                			self._set_grippers(open_left=False, open_right=False,
                                                               effort=-1)

                            			return outcomes.GRASP_PLAN_SUCCESS

                        		else:
                            			rospy.loginfo("It was not possible to grasp the object.")


        	# No grasps found
        self.loginfo("The object is not graspable.")
        self._tts.publish("The object is not graspable.")
        time.sleep(2)
        return outcomes.GRASP_PLAN_NONE

