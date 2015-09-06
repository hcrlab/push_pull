import outcomes
from pr2_pick_perception.msg import Box, Cluster2, BoundingBox
import rospy
import smach
from pr2_pick_main import handle_service_exceptions
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from actionlib import SimpleActionClient
import tf
import math
from geometry_msgs.msg import Point,PointStamped, Pose, PoseStamped
import visualization as viz
from visualization import IdTable
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String, Int32, Float32
from PushAwayExperiment import PushAwayExperiment
from pr2_grasp_evaluator import PushPullPlanner
from PullForwardExperiment import PullForwardExperiment
from PushSidewaysExperiment import PushSidewaysExperiment
from TopSidewaysExperiment import TopSidewaysExperiment
import time
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
import rospkg
from pr2_pick_contest.msg import Record, Trial, TrialParams 
from sensor_msgs.msg import PointCloud2, Image
from pr2_pick_perception.msg import Box, Cluster2, BoundingBox
from pr2_pick_contest.msg import Record
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest, PlanarPrincipalComponentsRequest, \
	DeleteStaticTransformRequest, BoxPointsResponse #, Cluster
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class CreatePlan(smach.State):

    name = 'CREATE_PLAN'

    def __init__(self, tts, tf_listener, **services):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcoomes=[outcomes.CREATE_PLAN_TOOL, outcomes.CREATE_PLAN_GRASP],
            input_keys=['debug', 'bounding_box', 'before_record', 'current_trial',
            'target_cluster', 'current_trial_num', 'planner'],
            output_keys=['tool_action', 'grasps'])

    def choose_action(self, state_tree):
        return action


    # @handle_service_exceptions(outcomes.MOVE_OBJECT_FAILURE)
    def execute(self, userdata):
        planner = userdata.planner
        state_tree = planner.generate_plan(userdata.before_record.bounding_box, userdata.before_record.pointcloud2)
        action = self.choose_action(state_tree)
        if isinstance(action, (int, long)):
            userdata.tool_action = action
            return outcomes.CREATE_PLAN_TOOL
        else:
            userdata.grasps = action
            return outcomes.CREATE_PLAN_GRASP



   
