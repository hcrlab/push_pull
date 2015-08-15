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
import rospkg

class PrepareToolAction(smach.State):

	name = 'PREPARE_TOOL_ACTION'

	## TODO: Eliminate this state completely

	def __init__(self, **services):
		smach.State.__init__(
			self,
			outcomes=[
				outcomes.PREPARE_TOOL_ACTION_SUCCESS,
				outcomes.PREPARE_TOOL_ACTION_FAILURE
			],
			input_keys=['trial_number', 'bin_id', 'debug', 'target_cluster', 'current_target',
						'item_model', 'target_descriptor', 're_grasp_attempt'],
			output_keys=['re_sense_attempt', 'previous_item', 'bounding_box_pose', 'bounding_box']
		)


	def save_image(self, image):
		self.bag_data.image = image

	@handle_service_exceptions(outcomes.PREPARE_TOOL_ACTION_FAILURE)
	def execute(self, userdata):



		return outcomes.PREPARE_TOOL_ACTION_SUCCESS
