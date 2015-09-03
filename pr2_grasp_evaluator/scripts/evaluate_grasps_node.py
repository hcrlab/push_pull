#!/usr/bin/env python

# Python Stuff
import glob
import copy
import itertools

# ROS Stuff
import actionlib
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, \
    Point, PointStamped, Vector3, Transform
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
import moveit_commander
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

# Local Stuff
from convert_pcl.srv import ConvertPCL
from joint_states_listener.srv import ReturnJointStates
from pr2_pick_contest.msg import Trial, Record
from pr2_pick_contest.srv import GetItems, SetItems, GetTargetItems
from pr2_pick_contest.srv import LookupItem
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms, GetGrippers, MoveArmIk
from pr2_pick_perception.msg import Object
from pr2_pick_perception.srv import CropShelf, CropShelfResponse, \
    DeleteStaticTransform, FindCentroid, LocalizeShelf, LocalizeShelfResponse, \
    SetStaticTransform, PlanarPrincipalComponents, GetItemDescriptor, ClassifyTargetItem
from pr2_pick_perception.srv import CountPointsInBox
from pr2_pick_perception.srv import SegmentItems
from pr2_pick_main import IdTable
from pr2_pick_main import publish_gripper, publish_bounding_box, publish_cluster
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class GripperBox:
    corners = []
    # min_x = None
    # max_x = None
    # min_y = None
    # max_y = None
    # min_z = None
    # max_z = None

class GraspEvaluator:
    def __init__(self):

        self.max_x_grasp_threshold = 0.9
        self.max_y_grasp_threshold = 0.185
        self.max_z_grasp_threshold = 1.14
        self.min_y_grasp_threshold = -0.185
        self.min_z_grasp_threshold = 0.76

        self.move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
        self.move_head = rospy.ServiceProxy('move_head_service', MoveHead)
        self.moveit_move_arm = rospy.ServiceProxy('moveit_service', MoveArm)
        self.move_arm_ik = rospy.ServiceProxy('move_arm_ik', MoveArmIk)
        self.set_grippers = rospy.ServiceProxy('set_grippers_service', SetGrippers)
        self.get_grippers = rospy.ServiceProxy('get_grippers_service', GetGrippers)
        self.tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
        self.joint_states_listener = rospy.ServiceProxy('return_joint_states', ReturnJointStates)
        self.attached_collision_objects = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)


        # World and Perception
        self.crop_shelf = rospy.ServiceProxy('perception/shelf_cropper', CropShelf)
        self.segment_items = rospy.ServiceProxy('perception/segment_items', SegmentItems)
        self.find_centroid = rospy.ServiceProxy('perception/find_centroid', FindCentroid)
        self.interactive_marker_server = InteractiveMarkerServer('pr2_pick_interactive_markers')
        self.localize_object = rospy.ServiceProxy('perception/localize_object',
                                              LocalizeShelf)
        self.markers = rospy.Publisher('pr2_pick_visualization', Marker)
        self.im_server = InteractiveMarkerServer('pr2_pick_interactive_markers')
        self.set_static_tf = rospy.ServiceProxy('perception/set_static_transform',
                                            SetStaticTransform)
        self.delete_static_tf = rospy.ServiceProxy('perception/delete_static_transform',
                                             DeleteStaticTransform)
        self.tf_listener = tf.TransformListener()
        self.get_planar_pca = rospy.ServiceProxy('planar_principal_components',
                                             PlanarPrincipalComponents)
        self.get_item_descriptor = rospy.ServiceProxy('perception/get_item_descriptor',
                                             GetItemDescriptor)
        self.classify_target_item = rospy.ServiceProxy('item_classifier/classify_target_item',
                                                   ClassifyTargetItem)
        self.count_points_in_box = rospy.ServiceProxy('perception/count_points_in_box',
                                                   CountPointsInBox)
        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        # Contest
        #self.get_items = rospy.ServiceProxy('inventory/get_items', GetItems),
        #'set_items = rospy.ServiceProxy('inventory/set_items', SetItems),
        self.get_target_items = rospy.ServiceProxy('inventory/get_target_items', GetTargetItems)
        self.lookup_item = rospy.ServiceProxy('item_database/lookup_item', LookupItem)
        self.convert_pcl_service = rospy.ServiceProxy('convert_pcl_service', ConvertPCL)
        self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene)

        self.arm_side = 'l'
        self.tool_name = 'tool'
        self.waypoint_duration = rospy.Duration(10.0)
        # approximate tool dimensions
        self.tool_x_size = 0.26
        self.tool_y_size = 0.01
        self.tool_z_size = 0.03
        self.joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upper_arm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
        ]
        # tool position relative to wrist_roll_link
        self.tool_x_pos = 0.29
        self.tool_y_pos = 0.0
        self.tool_z_pos = 0.0

        # approximately half hand thickness
        self.half_gripper_height = 0.03
        # approximate distance from palm frame origin to palm surface
        self.dist_to_palm = 0.12
        # approximate distance from palm frame origin to fingertip with gripper closed
        self.dist_to_fingertips = 0.21

        # approx dist between fingers when gripper open
        self.gripper_palm_width = 0.08

        # approx height of pads of fingertips
        self.gripper_finger_height = 0.03

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
        #rospy.loginfo("Result: {}".format(result))
        if not result or result.error_code.value != 0:
            return []
        return result.grasps

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

    def get_potential_grasps(self, trial):
        self.convert_pcl_service.wait_for_service()
        pc2_before = trial.before.pointcloud2
        # TODO: make sure the frame_id gets set when this is saved in the Record msg
        frame_id = 'bin_K' #pc2_before.header.frame_id
        points = pc2.read_points(pc2_before, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        if len(point_list) == 0:
            rospy.logwarn('[SenseBin]: Cluster with 0 points returned!')

        publish_cluster(self.markers, point_list, frame_id, 'experiment', 0)

        pc_before = self.convert_pcl_service(pc2_before).pointcloud
        pc_before.header.frame_id = frame_id
        
        # Set params for grasp planner
        self.call_set_params(overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = False, pregrasp_just_outside_box = True, backoff_depth_steps = 1)
        self.grasps_before = self.call_plan_point_cluster_grasp_action(pc_before,frame_id)
        rospy.loginfo("Number of grasps generated for before: " + str(len(self.grasps_before)))

        pc2_after = trial.after.pointcloud2
        frame_id = 'bin_K' #pc2_after.header.frame_id
        points = pc2.read_points(pc2_after, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        if len(point_list) == 0:
            rospy.logwarn('[SenseBin]: Cluster with 0 points returned!')

        publish_cluster(self.markers, point_list, frame_id, 'experiment', 1)

        pc_after = self.convert_pcl_service(pc2_after).pointcloud
        pc_after.header.frame_id = frame_id
        
        # Set params for grasp planner
        self.call_set_params(overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = False, pregrasp_just_outside_box = True, backoff_depth_steps = 1)
        self.grasps_after = self.call_plan_point_cluster_grasp_action(pc_after,frame_id)
        rospy.loginfo("Number of grasps generated for after: " + str(len(self.grasps_after)))

        publish_bounding_box(self.markers, trial.before.boundingbox.pose, 
                (trial.before.boundingbox.dimensions.x), 
                (trial.before.boundingbox.dimensions.y), 
                (trial.before.boundingbox.dimensions.z),
                0.0, 1.0, 0.0, 0.5, 20)

        publish_bounding_box(self.markers, trial.after.boundingbox.pose, 
                (trial.after.boundingbox.dimensions.x), 
                (trial.after.boundingbox.dimensions.y), 
                (trial.after.boundingbox.dimensions.z),
                1.0, 0.0, 0.0, 0.5, 21)

    def set_start_pose(self):

        rospy.loginfo("Setting start pose")

        # Hard code pre grasp state
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = "bin_K"
        pre_grasp_pose.pose.position.x = -0.40
        pre_grasp_pose.pose.position.y = 0.0
        pre_grasp_pose.pose.position.z = 0.23
        pre_grasp_pose.pose.orientation.x = 1.0
        pre_grasp_pose.pose.orientation.y = 0.0
        pre_grasp_pose.pose.orientation.z = 0.0
        pre_grasp_pose.pose.orientation.w = 0.0

        success_pre_grasp = self.moveit_move_arm(pre_grasp_pose, 
                                              0.005, 0.005, 12, 'left_arm',
                                              False, 1.0).success
    def add_shelf(self):
        rospy.loginfo("Adding shelf!")
        scene = moveit_commander.PlanningSceneInterface()
        shelf_odom = PoseStamped()
        shelf_odom.header.frame_id = 'base_footprint'
        shelf_odom.pose.position.x = 1.14
        shelf_odom.pose.position.y = 0.0
        shelf_odom.pose.position.z = 0.0
        shelf_odom.pose.orientation.x = 0.0
        shelf_odom.pose.orientation.y = 0.0
        shelf_odom.pose.orientation.z = 0.0
        shelf_odom.pose.orientation.w = 1.0

        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self.set_static_tf.wait_for_service()
    
        self.set_static_tf(transform)

        for i in range(10):
            scene.remove_world_object("bbox")
            scene.remove_world_object("shelf1")
            scene.remove_world_object("shelf")
            scene.remove_world_object("shelf2")
            scene.remove_world_object("shelf3")

        wall_pose1 = PoseStamped()
        wall_pose1.header.frame_id = "base_footprint"
        wall_pose1.pose.position.x = 0.9
        wall_pose1.pose.position.y = 0.025 - 0.2
        wall_pose1.pose.position.z = 0.94

        wall_pose2 = PoseStamped()
        wall_pose2.header.frame_id = "base_footprint"
        wall_pose2.pose.position.x = 0.9
        wall_pose2.pose.position.y = 0.385 - 0.2
        wall_pose2.pose.position.z = 0.94

        wall_pose3 = PoseStamped()
        wall_pose3.header.frame_id = "base_footprint"
        wall_pose3.pose.position.x = 0.9
        wall_pose3.pose.position.y = 0.20 - 0.2
        wall_pose3.pose.position.z = 1.14
        
        wall_pose4 = PoseStamped()
        wall_pose4.header.frame_id = "base_footprint"
        wall_pose4.pose.position.x = 0.9
        wall_pose4.pose.position.y = 0.20 - 0.2
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

        shelf_depth = 0.87

        top_row_z = 1.524
        second_row_z = 1.308
        third_row_z = 1.073
        bottom_row_z = .806

        left_column_y = .2921
        center_column_y = 0.0
        right_column_y = -.2921

        #TODO: Get rid of extra bin stuff
        bin_translations = {
            'A': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=top_row_z),
            'B': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=top_row_z),
            'C': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=top_row_z),
            'D': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=second_row_z),
            'E': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=second_row_z),
            'F': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=second_row_z),
            'G': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=third_row_z),
            'H': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=third_row_z),
            'I': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=third_row_z),
            'J': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=bottom_row_z),
            'K': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=bottom_row_z),
            'L': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=bottom_row_z),
        }

        for (bin_id, translation) in bin_translations.items():
       
            transform = TransformStamped(
                header=Header(frame_id='shelf',
                              stamp=rospy.Time.now(), ),
                transform=Transform(translation=translation,
                                    rotation=Quaternion(w=1,
                                                        x=0,
                                                        y=0,
                                                        z=0), ),
                child_frame_id='bin_{}'.format(bin_id), )
            self.set_static_tf.wait_for_service()
            self.set_static_tf(transform)


    def remove_outlier_grasps(self):
        rospy.loginfo("Removing outliers")
        filtered_before = []
        for grasp in self.grasps_before:

            grasp.grasp_pose.header.stamp = rospy.Time(0)

            self.tf_listener.waitForTransform('base_footprint', grasp.grasp_pose.header.frame_id, rospy.Time(0), rospy.Duration(15.0))

            grasp_pose = self.tf_listener.transformPose('base_footprint',
                                                                grasp.grasp_pose)
            if grasp_pose.pose.position.x > self.max_x_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.y > self.max_y_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.y < self.min_y_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.z > self.max_z_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.z < self.min_z_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue

            filtered_before.append(grasp)

        self.grasps_before = filtered_before

        filtered_after = []
        for grasp in self.grasps_after:
            grasp.grasp_pose.header.stamp = rospy.Time(0)

            self.tf_listener.waitForTransform('base_footprint', grasp.grasp_pose.header.frame_id, rospy.Time(0), rospy.Duration(15.0))

            grasp_pose = self.tf_listener.transformPose('base_footprint',
                                                                grasp.grasp_pose)
            if grasp_pose.pose.position.x > self.max_x_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.y > self.max_y_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.y < self.min_y_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.z > self.max_z_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue
            elif grasp_pose.pose.position.z < self.min_z_grasp_threshold:
                rospy.loginfo("Removing grasp")
                continue

            filtered_after.append(grasp)
            
        self.grasps_after = filtered_after

    def remove_shelf_intersections(self):
        rospy.loginfo("Removing grasps that intersect with shelf")
        rospy.loginfo("Number before grasps left: {}".format(len(self.grasps_before)))
        rospy.loginfo("Number after grasps left: {}".format(len(self.grasps_after)))
        self.grasps_before = self.find_shelf_intersections(self.grasps_before)
        self.grasps_after = self.find_shelf_intersections(self.grasps_after)

    def find_shelf_intersections(self, grasps):
        filtered = []

        for grasp in grasps:
            grasp_pose = grasp.grasp_pose

            transform = TransformStamped()
            transform.header.frame_id = grasp_pose.header.frame_id
            transform.header.stamp = rospy.Time.now()
            transform.transform.translation = grasp_pose.pose.position
            transform.transform.rotation = grasp_pose.pose.orientation
            transform.child_frame_id = 'grasp'
            self.set_static_tf.wait_for_service()
            self.set_static_tf(transform)
            rospy.sleep(0.25)

            y_offset = 0.005

            gripper = GripperBox()
            min_x = 0.0
            max_x = self.dist_to_fingertips
            min_y = -1 * self.gripper_palm_width/2 + y_offset
            max_y = self.gripper_palm_width/2 + y_offset
            min_z = -1 * self.gripper_finger_height/2
            max_z = self.gripper_finger_height/2

            x_bounds = [min_x, max_x]
            y_bounds = [min_y, max_y]
            z_bounds = [min_z, max_z]

            all_bounds = [x_bounds, y_bounds, z_bounds]
            for bounds in itertools.product(*all_bounds):
                corner = PointStamped()
                corner.header.frame_id = grasp_pose.header.frame_id
                corner.point.x = bounds[0]
                corner.point.y = bounds[1]
                corner.point.z = bounds[2]
                corner_base_footprint = self.tf_listener.transformPoint('base_footprint',
                                                                        corner)
                gripper.corners.append(copy.deepcopy(corner_base_footprint))
            
            palm_pose = PoseStamped()
            palm_pose.header.frame_id = 'grasp'
            palm_pose.pose.position.x = min_x +\
                             (max_x - min_x) / 2
            palm_pose.pose.position.y = min_y +\
                             (max_y - min_y) / 2
            palm_pose.pose.position.z = min_z +\
                             (max_z - min_z) / 2

            publish_bounding_box(self.markers, palm_pose, 
                (max_x - min_x), 
                (max_y - min_y), 
                (max_z - min_z),
                0.0, 0.0, 1.0, 0.5, 4)

            add = True

            for corner in gripper.corners:
                # Check intersection with right shelf wall
                if (corner.point.x > 0.71) and (corner.point.y < -0.185) and (corner.point.z > 0.78) and (corner.point.z < 1.14):
                    rospy.loginfo("Removing grasp because (corner.point.x > 0.71) and (corner.point.y < 0.025) and (corner.point.z > 0.78) and (corner.point.z < 1.14)")
                    rospy.loginfo("Corner: {}".format(corner))
                    add = False
                    #rospy.sleep(10.0)
                    break 
                # Check intersection with left shelf wall
                elif (corner.point.x > 0.71) and (corner.point.y > 0.185) and (corner.point.z > 0.78) and (corner.point.z < 1.14):
                    rospy.loginfo("Removing grasp because (corner.point.x > 0.71) and (corner.point.y > 0.385) and (corner.point.z > 0.78) and (corner.point.z < 1.14)")
                    rospy.loginfo("Corner: {}".format(corner))
                    add = False
                    #rospy.sleep(10.0)
                    break
                # Check intersection with bottom shelf wall
                elif (corner.point.x > 0.71) and (corner.point.z < 0.78):
                    rospy.loginfo("Removing grasp because (corner.point.x > 0.71) and (corner.point.z < 0.78)")
                    rospy.loginfo("Corner: {}".format(corner))
                    add = False
                    #rospy.sleep(10.0)
                    break
                # Check intersection with top shelf wall
                elif (corner.point.x > 0.71) and (corner.point.z > 1.14):
                    rospy.loginfo("Removing grasp because (corner.point.x > 0.71) and (corner.point.z > 1.14)")
                    rospy.loginfo("Corner: {}".format(corner))
                    add = False
                    #rospy.sleep(10.0)
                    break
                # else:
                #     rospy.loginfo("Not removing grasp")
                #     rospy.loginfo("Corner: {}".format(corner))
                #     #rospy.sleep(10.0)
                #     filtered.append(grasp)
                #     break

            if add:
                filtered.append(grasp)

        return filtered

    def remove_unreachable(self):
        rospy.loginfo("Removing unreachable grasps")
        rospy.loginfo("Number before grasps left: {}".format(len(self.grasps_before)))
        rospy.loginfo("Number after grasps left: {}".format(len(self.grasps_after)))

        filtered_before = []
        for grasp in self.grasps_before:
            #self.set_start_pose()
            for i in range(10):     
                publish_gripper(self.im_server, grasp.grasp_pose, 'grasp_target')
                    # Test if grasp is going to hit the shelf
            success_grasp = self.moveit_move_arm(grasp.grasp_pose,
                                                0.005, 0.005, 6, 'left_arm',
                                                True, 1.0).success  
            if success_grasp:
                filtered_before.append(grasp)
            else:
                rospy.loginfo("Removing grasp")
        self.grasps_before = filtered_before

        filtered_after = []
        for grasp in self.grasps_after:
            #self.set_start_pose()
            for i in range(10):     
                publish_gripper(self.im_server, grasp.grasp_pose, 'grasp_target')
                    # Test if grasp is going to hit the shelf
            success_grasp = self.moveit_move_arm(grasp.grasp_pose,
                                                0.005, 0.005, 6, 'left_arm',
                                                True, 1.0).success  
            if success_grasp:
                filtered_after.append(grasp)
            else:
                rospy.loginfo("Removing grasp")
        
        self.grasps_after = filtered_after

    def move_arms_to_side(self):

        rospy.loginfo("Moving arms out of way")

        scene = moveit_commander.PlanningSceneInterface()
        

        for i in range(10):
            scene.remove_world_object("bbox")
            scene.remove_world_object("shelf1")
            scene.remove_world_object("shelf")
            scene.remove_world_object("shelf2")
            scene.remove_world_object("shelf3")


        pose_target = Pose()
        quaternion = tf.transformations.quaternion_from_euler(-2.119,1.357,1.969)

        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]
        pose_target.position.x = 0.3
        pose_target.position.y = 0.42 
        pose_target.position.z = 0.98

        posestamped = PoseStamped()
        posestamped.pose = pose_target
        posestamped.header.frame_id = 'base_footprint'
            

        self.moveit_move_arm(posestamped,
                                                0.005, 0.005, 12, 'left_arm',
                                                False, 1.0).success 

        pose_target = Pose()
            
        quaternion = tf.transformations.quaternion_from_euler(-2.989, 0.065, 0.895)

        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]
        pose_target.position.x = 0.30
        pose_target.position.y = -0.42 
        pose_target.position.z = 0.98

        posestamped = PoseStamped()
        posestamped.pose = pose_target
        posestamped.header.frame_id = 'base_footprint'
            

        self.moveit_move_arm(posestamped,
                                                0.005, 0.005, 12, 'right_arm',
                                                False, 1.0).success 


if __name__ == '__main__':
    rospy.init_node('grasp_evaluator_node')

    ge = GraspEvaluator()
    ge.move_arms_to_side()
    ge.add_shelf()
    ge.set_start_pose()
    path = raw_input("Please enter the path to folder with the bag files: ")
    file_list = glob.glob( path + '/*.bag')
    for file_name in file_list:
        if file_name[-13:] == 'evaluated.bag':
            continue
        rospy.loginfo("Current file: " + file_name)
        bag = rosbag.Bag(file_name)
        for topic, trial_msg, t in bag.read_messages():
            # Use Grasp Planner to Generate Many Potential Grasps
            ge.get_potential_grasps(trial_msg)
            # Get Rid of ones that are outside the bounds of the shelf (in y,z direction mostly)
            ge.remove_outlier_grasps()
            # Remove grasps that intersect with the shelf.
            ge.remove_shelf_intersections()
            # Remove non-reachable
            ge.remove_unreachable()

            evaluated_bag = rosbag.Bag(file_name[:-4] + '_evaluated.bag' , 'w')
            trial_msg.before.grasps = ge.grasps_before
            if len(ge.grasps_before) > 0:
                trial_msg.before.is_graspable = True
            else:
                trial_msg.before.is_graspable = False
            trial_msg.after.grasps = ge.grasps_after
            if len(ge.grasps_after) > 0:
                trial_msg.after.is_graspable = True
            else:
                trial_msg.after.is_graspable = False
            trial_msg.after.grasps = ge.grasps_after
            evaluated_bag.write('trial', trial_msg)
            evaluated_bag.close()

        bag.close()
    # rospy.spin()
