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

class ExecuteGrasp(smach.State):

    name = 'EXECUTE_GRASP'

    def __init__(self, tts, tf_listener, **services):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcoomes=[outcomes.EXECUTE_GRASP_SUCCESS, outcomes.EXECUTE_GRASP_FAILURE],
            input_keys=['debug', 'grasps'],
            output_keys=[])


    # @handle_service_exceptions(outcomes.MOVE_OBJECT_FAILURE)
    def execute(self, userdata):
        grasp_poses = userdata.grasps
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
                        return outcomes.EXECUTE_GRASP_SUCCESS

                    else:

                        rospy.loginfo("It was not possible to grasp the object.")
                        return outcomes.EXECUTE_GRASP_FAILURE



   
