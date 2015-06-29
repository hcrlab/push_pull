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

def dummy(idx, box, num_points, transformed_point):
        if (transformed_point.point.x >= box.min_x and
                transformed_point.point.x < box.max_x and
                transformed_point.point.y >= box.min_y and
                transformed_point.point.y < box.max_y and
                transformed_point.point.z >= box.min_z and
                transformed_point.point.z < box.max_z):
                num_points[idx] += 1

def draw_grasps(grasps, frame, ns = 'grasps', pause = 0, frame_locked = False):

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
    pre_grasp_x_distance = 0.37

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
            output_keys=['re_sense_attempt', 'previous_item']
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


    def loginfo(self, string):
        #self.debug_grasp_pub.publish(string)
        rospy.loginfo(string)

    def check_pose_within_bounds(self, pose, shelf_bottom_height, shelf_height, 
                                    shelf_width, bin_id, frame, rotated):
        """
        Check if a pose is within the width and height of the bin
        i.e. That it will not hit the shelf
        """
	viz.publish_gripper(self._im_server, pose, 'grasp_target')



        in_bounds = True
        pose_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                pose)

        # Check if within bin_width
        if pose_in_bin_frame.pose.position.y > ((self.shelf_width/2) - 
            (self.gripper_palm_width + self.bin_bound_left)/2):
            in_bounds = False
            #self.loginfo("Pose >  y bounds")
            #self.loginfo("Y pose: {}".format(pose_in_bin_frame.pose.position.y))
            #self.loginfo("Bound: {}".format(((self.shelf_width/2) - 
            #    (self.gripper_palm_width + self.bin_bound_left)/2)))

        elif pose_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + 
            (self.gripper_palm_width + self.bin_bound_right)/2):
            in_bounds = False
            #self.loginfo("Pose <  y bounds")
            #self.loginfo("Y pose: {}".format(pose_in_bin_frame.pose.position.y))
            #self.loginfo("Bound: {}".format(-1*self.shelf_width/2 + 
            #    (self.gripper_palm_width + self.bin_bound_right)/2))
        pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        pose_in_bin_frame)

        if rotated:
            gripper_offset = self.gripper_palm_width/2
        else:
            gripper_offset = self.half_gripper_height

        if ((pose_in_base_footprint.pose.position.z > 
                (self._shelf_bottom_height_g_i + 2*gripper_offset))
                and (pose_in_base_footprint.pose.position.z < 
                (self._shelf_bottom_height_g_i + self.shelf_height - (gripper_offset)))):
            pose_in_base_footprint.pose.position.z = pose_in_base_footprint.pose.position.z
        else:
            #if pose_in_bin_frame.pose.position.x > 0:
            in_bounds = False
            #self.loginfo("Pose not within z bounds")
            #self.loginfo("Z pose: {}".format(pose_in_base_footprint.pose.position.z))
            #self.loginfo("Bound: {}".format((self.shelf_bottom_height + gripper_offset)))
            #self.loginfo("Bound: {}".format((self.shelf_bottom_height + 
            #                            self.shelf_height - gripper_offset)))

        pose_in_frame = self._tf_listener.transformPose(frame,
                                                        pose_in_bin_frame)

        return in_bounds


    def filter_grasps(self, grasps):
        grasps_in_bound = deepcopy(grasps)
        for grasp in grasps:
            grasp_in_bounds = self.check_pose_within_bounds(grasp.grasp_pose, 
                                                            self._shelf_bottom_heights['H'], 
                                                            self._shelf_heights['H'], 
                                                            self._shelf_widths['H'], 
                                                            'H', 
                                                            'base_footprint', False)
            if(grasp_in_bounds == False):
                grasps_in_bound.remove(grasp)

        return grasps_in_bound

    def execute_grasp(self, grasps, item_model):
        """
        Execute good grasp
        Tries to use Moveit motion planning to execute pre-grasp
        Uses IK (no collision checking) to execute grasp
        Tries best grasp first and if that fails, tries subsequent ones until none left
        """
               
        success_pre_grasp = False
        success_grasp = False
        self.loginfo("Executing grasp")
        for grasp in grasps:
            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')

            if self._debug:
                raw_input('(Debug) Press enter to continue >')

            self.loginfo("Pre_grasp: {}".format(grasp["pre_grasp"]))
            self.loginfo("Grasp: {}".format(grasp["grasp"]))
            self._moveit_move_arm.wait_for_service()

            self.loginfo("Service available")

            if not self.top_shelf:

                for i in range(self.pre_grasp_attempts):
                    success_pre_grasp = self._moveit_move_arm(grasp["pre_grasp"], 
                                                        0.005, 0.005, 12, 'right_arm',
                                                        False).success
                    if success_pre_grasp:
                        break
            else:
                self.loginfo("Waiting for ik service")
                self._move_arm_ik.wait_for_service()
                self.loginfo("Service available")
                success_pre_grasp = self._move_arm_ik(grasp["pre_grasp"], 
                                        MoveArmIkRequest().RIGHT_ARM, rospy.Duration(5)).success


            if not success_pre_grasp:
                continue
            else:
                self.loginfo('Pre-grasp succeeeded')
                self.loginfo('Open Hand')
                self._set_grippers.wait_for_service()
 
                grippers_open = self._set_grippers(False, True, -1)

            if self._debug:
                raw_input('(Debug) Press enter to continue >')

            #self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene) 
            #rospy.wait_for_service('/get_planning_scene', 10.0) 
            #get_planning_scene = rospy.ServiceProxy('/get_planning_scene', 
            #                                        GetPlanningScene) 
            #request = PlanningSceneComponents(
            #            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX) 
            #response = get_planning_scene(request) 

            #acm = response.scene.allowed_collision_matrix 
            #if not 'bbox' in acm.default_entry_names: 
            #    # add button to allowed collision matrix 
            #    acm.default_entry_names += ['bbox'] 
            #    acm.default_entry_values += [True] 

            #    planning_scene_diff = PlanningScene( 
            #        is_diff=True, 
            #        allowed_collision_matrix=acm) 

            #    #self._pubPlanningScene.publish(planning_scene_diff) 
            #    #rospy.sleep(1.0)


            self._move_arm_ik.wait_for_service()

            success_grasp = self._move_arm_ik(grasp["grasp"], 
                                MoveArmIkRequest().RIGHT_ARM, rospy.Duration(5)).success

            #self._moveit_move_arm.wait_for_service()
            #success_grasp = self._moveit_move_arm(grasp["grasp"], 
            #                                        0.01, 0.01, 0, 'right_arm',
            #                                        False).success
 

            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            if success_grasp:
                self.loginfo('Grasp succeeded')
                self.loginfo('Close Hand')
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(open_left=False, open_right=False,
                                                   effort=item_model.grasp_effort)
                gripper_states = self._get_grippers()
                if not gripper_states.right_open:
                    self._set_grippers(open_left=False, open_right=False,
                                                   effort=-1)

                break
        if success_grasp:
            return True
        else:
            return False

    def call_set_params(self, side_step = 0.02, palm_step = 0.005, overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = True, pregrasp_just_outside_box = False, backoff_depth_steps = 5):
        
        req = SetPointClusterGraspParamsRequest()
        req.height_good_for_side_grasps = 0.05
        req.gripper_opening = 0.083
        req.side_step = 0.01
        req.palm_step = palm_step
        req.overhead_grasps_only = False
        req.side_grasps_only = False
        req.include_high_point_grasps = include_high_point_grasps
        req.pregrasp_just_outside_box = True
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

    def create_grasps(self, grasp, boudingbox_pose):

        object_pose = boudingbox_pose
        object_pose = self._tf_listener.transformPose('base_footprint',
                                                                object_pose)
        self.shelf_bottom_height = self._shelf_bottom_heights['H']

        pre_grasp_pose_target = PoseStamped()
        pre_grasp_pose_target.header.frame_id = 'base_footprint';

        pre_grasp_pose_target.pose.orientation.w = 1
        pre_grasp_pose_target.pose.position.x = 0.37 
        pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y

        grasp_pose_target = PoseStamped()
        grasp_pose_target.header.frame_id = 'base_footprint';

        grasp_pose_target.pose = grasp.grasp_pose

        return pre_grasp_pose_target, grasp_pose_target

    @handle_service_exceptions(outcomes.GRASP_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Started Grasp Planner")


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
	rospy.loginfo("Current Target in GraspPlanner: " + userdata.current_target)
	rospy.loginfo("Waiting for convert_pcl service")
        self.convert_pcl.wait_for_service()
        rospy.loginfo("PCL service found")
	self._cluster2 = Cluster2()
	self._cluster2.pointcloud = self.convert_pcl(userdata.target_cluster.pointcloud).pointcloud        
	self._cluster = userdata.target_cluster
	self._cluster2.header = userdata.target_cluster.header
	self._cluster2.pointcloud.header = userdata.target_cluster.header
	self._cluster2.id = userdata.target_cluster.id
	rospy.loginfo("Cluster frame id: " + self._cluster.header.frame_id)
	rospy.loginfo("Poincloud frame id: " + self._cluster2.header.frame_id)
	rospy.loginfo("Conversion ended")
        rospy.loginfo(type(self._cluster2))
	rospy.loginfo(type(self._cluster))

        # self._cluster = userdata.target_cluster.pointcloud
        # #rospy.loginfo("CLUSTER:")
        # rospy.loginfo(type(self._cluster))
        tf_broadcaster = tf.TransformBroadcaster()
        tf_listener = tf.TransformListener()

        # #set params for planner (change to try different settings)
        self.call_set_params(overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = False, pregrasp_just_outside_box = True, backoff_depth_steps = 1)

        (box_pose, box_dims) = self.call_find_cluster_bounding_box(self._cluster2.pointcloud)
	box_pose.header.frame_id = self._cluster.header.frame_id
        viz.publish_bounding_box(self._markers, box_pose, 
                     (box_dims.x), 
                     (box_dims.y), 
                     (box_dims.z),
                     1.0, 0.0, 0.0, 0.5, 1)

        #grasps = self.call_plan_point_cluster_grasp(self._cluster2.pointcloud)
        grasps = self.call_plan_point_cluster_grasp_action(self._cluster2.pointcloud,self._cluster.header.frame_id )
        #grasp_poses = [grasp.grasp_pose for grasp in grasps]
        # #rospy.loginfo(grasps)

        rospy.loginfo("Number of grasps: ")
        rospy.loginfo(len(grasps))
        #grasp_not_stamped = []
        #for pose_stamped in grasp_poses:
        #    grasp_not_stamped.append(pose_stamped.pose)
	#rospy.loginfo(grasp_not_stamped)
        #rospy.loginfo("Cluster frame id: " + self._cluster.header.frame_id)
	
        #grasps = self.filter_grasps(grasps)
        rospy.loginfo("Number of grasps after filter: ")
	rospy.loginfo( len(grasps))
	grasp_poses = [grasp.grasp_pose for grasp in grasps]
	grasp_not_stamped = []
        for pose_stamped in grasp_poses:
            grasp_not_stamped.append(pose_stamped.pose)


	if(len(grasps) > 0):
	    draw_grasps(grasp_not_stamped, self._cluster.header.frame_id, pause = 0)
	    for grasp in grasp_poses:
            	viz.publish_gripper(self._im_server, grasp , 'grasp_target') 
                pre_grasp_target, grasp_target = self.create_grasps(grasp, box_pose)
                moveit_move_arm.wait_for_service()

                success_pre_grasp = self._moveit_move_arm(pre_grasp_target, 
                                                        0.005, 0.005, 12, 'right_arm',
                                                        False).success
                break
		#raw_input("Press enter to see another grasp")
	    #success_grasp = self.execute_grasp(grasp_poses, userdata.item_model)
            #success_pre_grasp = self._moveit_move_arm(grasp["pre_grasp"], 
                                                        #0.005, 0.005, 12, 'right_arm',
                                                        #False).success
	    self._tts.publish("The object is graspable.")
	    time.sleep(2) 
            self.loginfo("The object is graspable.")
            success_grasp = True
        else:
            self._tts.publish("The object is not graspable.")
            time.sleep(2)
	    self.loginfo("The object is not graspable.")
	# for grasp in grasp_poses:
	# 	rospy.loginfo(type(grasp))
	# 	viz.publish_gripper(self._im_server, grasp, 'grasp_target')
	# 	raw_input("Enter for next grasp")
        return outcomes.GRASP_PLAN_SUCCESS


        # start = datetime.datetime.now()

        # # Delete any leftover transforms from previous runs
        # bin_ids = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
        # for bin_id in bin_ids:
        #     self._delete_static_tf.wait_for_service()
        #     req = DeleteStaticTransformRequest()
        #     req.parent_frame = "bin_" + str(bin_id)
        #     req.child_frame = "object_axis"
        #     self._delete_static_tf(req)
        #     req = DeleteStaticTransformRequest()
        #     req.child_frame = "bin_" + str(bin_id)
        #     req.parent_frame = "object_axis"
        #     self._delete_static_tf(req)
        #     req = DeleteStaticTransformRequest()
        #     req.child_frame = "bin_" + str(bin_id)
        #     req.parent_frame = "bounding_box"
        #     self._delete_static_tf(req)


        # self._delete_static_tf.wait_for_service()
        # req = DeleteStaticTransformRequest()
        # req.parent_frame = "base_footprint"
        # req.child_frame = "head_yaw"
        # self._delete_static_tf(req)

        # # Set a bunch of member variables
        # # These could be way better organised and consistently named! Sorry!
        # if userdata.bin_id < 'D':
        #     self.top_shelf = True
        # else:
        #     self.top_shelf = False

        # self.shelf_width = self._shelf_widths[userdata.bin_id]
        # self.shelf_height = self._shelf_heights[userdata.bin_id]
        # self.bin_id = userdata.bin_id
        # self._debug = userdata.debug
        # self.allowed_grasps = userdata.item_model.allowed_grasps
        # self.target_descriptor = userdata.target_descriptor
        # self.bin_bound_left = self._bin_bounds_left[userdata.bin_id] 
        # self.bin_bound_right = self._bin_bounds_right[userdata.bin_id]
        # self.grasp_multiple_heights = userdata.item_model.grasp_multiple_heights
        # self.grasp_wide_end = userdata.item_model.grasp_wide_end
        # self._cluster = userdata.target_cluster

        # self.downsampled_cluster = self.downsample_cluster(self._cluster)

        # rospy.loginfo("Downsampled cluster size: " + str(len(self.downsampled_cluster)))

        # if userdata.item_model.allow_finger_collisions:
        #     self.max_finger_collision_points = 1000

        # self._tts.publish('Grasping {}'.format(userdata.item_model.speech_name))
        # self._tuck_arms.wait_for_service()
        # tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)

        # # # Get the pose of the target item in the base frame
        # # response = self._find_centroid(userdata.target_cluster)
        # # item_point = response.centroid
        # # item_pose = PoseStamped(
        # #     header=Header(
        # #         frame_id=item_point.header.frame_id,
        # #         stamp=rospy.Time(0),
        # #     ),
        # #     pose=Pose(
        # #         position=item_point.point,
        # #         orientation=Quaternion(w=1, x=0, y=0, z=0),
        # #     )
        # # )

        # self._tf_listener.waitForTransform("base_footprint", 
        #         userdata.target_descriptor.planar_bounding_box.pose.header.frame_id, 
        #         rospy.Time(0), rospy.Duration(10.0))

        # base_frame_item_pose = self._tf_listener.transformPose('base_footprint',
        #                             userdata.target_descriptor.planar_bounding_box.pose)

        # # self.loginfo(
        # #     'Grasping item in bin {} from pose {}'
        # #     .format(userdata.bin_id, base_frame_item_pose)
        # # )
        # # if userdata.debug:
        # #     raw_input('(Debug) Press enter to continue >')

        # # Add shelf and bounding box to planning scene
        # planar_bounding_box = userdata.target_descriptor.planar_bounding_box
        # scene = moveit_commander.PlanningSceneInterface()
        # scene.remove_world_object('bbox')
        # scene.remove_world_object("shelf")

        # self.add_shelf_mesh_to_scene(scene)

        # # Weird loop that you have to do because Moveit it weird
        # for i in range(10):
        #     #self.loginfo("MoveIt looping hack, iteration %d" % i)
        #     scene.add_box("bbox", planar_bounding_box.pose, 
        #                 (planar_bounding_box.dimensions.x, 
        #                 planar_bounding_box.dimensions.y, 
        #                 planar_bounding_box.dimensions.z))
        #     rospy.sleep(0.1)

        # # Make a frame that has the same yaw as head frame
        # #self.loginfo("About to wait for transform")
        # (trans, rot) = self._tf_listener.lookupTransform("base_footprint", "head_mount_link", rospy.Time(0))
        # #self.loginfo("Tranform: {}, {}".format(trans, rot))

        # quaternion = (
        #     rot[0],
        #     rot[1],
        #     rot[2],
        #     rot[3])
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = 0
        # pitch = 0
        # yaw = euler[2] 

        # pose = PoseStamped()

        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # pose.pose.orientation.x = quaternion[0]
        # pose.pose.orientation.y = quaternion[1]
        # pose.pose.orientation.z = quaternion[2]
        # pose.pose.orientation.w = quaternion[3]
        
        # pose.pose.position.x = trans[0]
        # pose.pose.position.y = trans[1]
        # pose.pose.position.z = trans[2]

        # transform = TransformStamped()
        # transform.header.frame_id = 'base_footprint'
        # transform.header.stamp = rospy.Time.now()
        # transform.transform.translation = pose.pose.position
        # transform.transform.rotation = pose.pose.orientation
        # transform.child_frame_id = 'head_yaw'
        # self._set_static_tf.wait_for_service()
        # self._set_static_tf(transform)
        # rospy.sleep(0.25)

        # transform = TransformStamped()
        # transform.header.frame_id = planar_bounding_box.pose.header.frame_id
        # transform.header.stamp = rospy.Time.now()
        # transform.transform.translation = planar_bounding_box.pose.pose.position
        # transform.transform.rotation = planar_bounding_box.pose.pose.orientation
        # transform.child_frame_id = 'bounding_box'
        # self._set_static_tf.wait_for_service()
        # self._set_static_tf(transform)
        # rospy.sleep(0.25)


        # viz.publish_bounding_box(self._markers,  planar_bounding_box.pose, planar_bounding_box.dimensions.x, planar_bounding_box.dimensions.y, planar_bounding_box.dimensions.z,
        #     1.0, 0.0, 0.0, 0.5, 25)
        # grasp_gen_start = datetime.datetime.now()
        # grasping_pairs = self.generate_grasps(planar_bounding_box, userdata.bin_id)
        # grasp_gen_end = datetime.datetime.now()
        # rospy.loginfo("Grasp gen: " + str((grasp_gen_end - grasp_gen_start).total_seconds()))
        # filter_start = datetime.datetime.now()
        # filtered_grasps = self.filter_grasps(grasping_pairs)
        # filter_end = datetime.datetime.now()
        # rospy.loginfo("Filter: " + str((filter_end - filter_start).total_seconds()))

        # self._delete_static_tf.wait_for_service()
        # req = DeleteStaticTransformRequest()
        # req.parent_frame = "bin_" + str(self.bin_id)
        # req.child_frame = "object_axis"
        # self._delete_static_tf(req)

        # self._delete_static_tf.wait_for_service()
        # req = DeleteStaticTransformRequest()
        # req.parent_frame = "base_footprint"
        # req.child_frame = "head_yaw"
        # self._delete_static_tf(req)

        # self._delete_static_tf.wait_for_service()
        # req = DeleteStaticTransformRequest()
        # req.parent_frame = planar_bounding_box.pose.header.frame_id
        # req.child_frame = "bounding_box"
        # self._delete_static_tf(req)
        # end = datetime.datetime.now()
        # self.timer1 += (end - start).total_seconds()
        # rospy.loginfo("Total: " + str(self.timer1))
        # rospy.loginfo("Intersection Function: " + str(self.timer))
        # rospy.loginfo("IK/Moveit Functiona: " + str(self.timer2))


        # if len(filtered_grasps) > 0:
        #     #success_grasp = self.execute_grasp(filtered_grasps, userdata.item_model)
        #     self.__tts.publish("The object is graspable.")
        #     self.loginfo("The object is graspable.")
        #     success_grasp = True
        # else:
        #     #end = datetime.datetime.now()
        #     #self.timer1 += int((end - start).total_seconds())
        #     #rospy.loginfo("Total: " + str(self.timer1/1e6))
        #     #rospy.loginfo("Function: " + str(self.timer/1e6))
        #     self.loginfo('No good grasps found')
        #     self._tts.publish('No grasps found.')
        #     # if not userdata.re_grasp_attempt:
        #     #     userdata.re_sense_attempt = True
        #     #     return outcomes.GRASP_NONE
        #     # else:
        #     #     userdata.re_sense_attempt = False
        #     #     return outcomes.GRASP_FAILURE


        # if self._debug:
        #         raw_input('(Debug) Press enter to continue >')

        # if not success_grasp:
        #     self.loginfo('Grasping failed')
        #     self._tts.publish('Grasping failed. Giving up.')
        #     userdata.re_sense_attempt = False
        #     return outcomes.GRASP_FAILURE

        # else:
        #     self.loginfo('Grasping succeeded')
        #     self._tts.publish('Grasping succeeded.')
        #     userdata.re_sense_attempt = False
            # return outcomes.GRASP_SUCCESS
