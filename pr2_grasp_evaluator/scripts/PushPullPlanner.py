#!/usr/bin/env python

# Python Stuff
import glob
import copy
import itertools
import math

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
import rospkg
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
from Regressor import Regressor
from GraspEvaluator import GraspEvaluator
from pr2_grasp_evaluator.srv import TransformPointCloud, TransformPointCloudRequest
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms, GetGrippers, MoveArmIk
from pr2_pick_perception.msg import Object
from pr2_pick_perception.srv import CropShelf, CropShelfResponse, \
    DeleteStaticTransform, FindCentroid, LocalizeShelf, LocalizeShelfResponse, \
    SetStaticTransform, PlanarPrincipalComponents, GetItemDescriptor, ClassifyTargetItem
from pr2_pick_perception.srv import CountPointsInBox
from pr2_pick_perception.srv import SegmentItems
from pr2_pick_perception.srv import DeleteStaticTransformRequest
from pr2_pick_main import IdTable
from pr2_pick_main import publish_gripper, publish_bounding_box, publish_cluster
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class State(object):
    def __init__(self, bounding_box, point_cloud):
        self.children = []
        self.bounding_box = bounding_box
        self.point_cloud = point_cloud
        self.grasps = []
        self.actions = []
        self.tipped = False

    def add_child(self, obj):
        self.children.append(obj)

    def get_children(self):
        return self.children

    def add_grasps(self, grasps):
        self.grasps = grasps
    def add_actions(self, action, prev_actions):
        rospy.loginfo("Prev actions: {}".format(prev_actions))
        rospy.loginfo("Current action: {}".format(action))
        actions = copy.deepcopy(prev_actions)
        actions.append(copy.copy(action))
        self.actions = actions
    def get_actions(self):
        return self.actions

class PushPullPlanner:
    def __init__(self):
        self.regressor = None
        self.max_depth = 2
        self.max_nodes = 111
        self.tf_listener = tf.TransformListener()
        self.set_static_tf = rospy.ServiceProxy('perception/set_static_transform',
                                            SetStaticTransform)
        self.delete_static_tf = rospy.ServiceProxy('perception/delete_static_transform',
                                             DeleteStaticTransform)
        self.transform_point_cloud_service = rospy.ServiceProxy('/transform_point_cloud/transform_point_cloud', TransformPointCloud)
        self.markers = rospy.Publisher('pr2_pick_visualization', Marker)
        self.actions = ["front_center_push",
                        "front_side_push_r",
                        "front_side_push_l",
                        "side_push_r",
                        "side_push_l",
                        "side_rotate_r",
                        "side_rotate_l",
                        "top_pull",
                        "top_sideward_pull_r",
                        "top_sideward_pull_l"]

        self.grasp_evaluator = GraspEvaluator()

    def train(self, path_to_data):
        self.regressor = Regressor(path_to_data)
        self.regressor.train()

    def get_depth(self, tree):
        count = 1
        current_node = tree
        while True:
            if current_node.get_children():
                count +=1
                current_node = current_node.get_children()[9]
            else:
                break
        return count

    def transform_point_cloud(self, point_cloud, pose, new_x, new_y, new_yaw):
        # Make tf frame at original bounding box origin


        points = pc2.read_points(point_cloud, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]

        publish_cluster(self.markers, point_list, 'bin_K', 'experiment', 0)
        
        transform = TransformStamped()
        transform.header.frame_id = 'bin_K'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = copy.deepcopy(pose.pose.position)
        transform.transform.rotation = copy.deepcopy(pose.pose.orientation)
        transform.child_frame_id = 'bounding_box'

        self.set_static_tf.wait_for_service()
    
        self.set_static_tf(transform)

        # tranform points into that frame
        self.transform_point_cloud_service.wait_for_service()
        resp = self.transform_point_cloud_service(point_cloud, 'bin_K', 'bounding_box')



        # Move tf frame to new bounding box origin
        self.delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "bin_K"
        req.child_frame = "bounding_box"
        self.delete_static_tf(req)

        transform = TransformStamped()
        transform.header.frame_id = 'bin_K'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation.x = new_x
        transform.transform.translation.y = new_y
        transform.transform.translation.z = pose.pose.position.z

        q = tf.transformations.quaternion_from_euler(0, 0, new_yaw)
        #type(pose) = geometry_msgs.msg.Pose
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        transform.child_frame_id = 'bounding_box'

        self.set_static_tf.wait_for_service()
    
        self.set_static_tf(transform)



        # tranform points into that frame
        self.transform_point_cloud_service.wait_for_service()
        resp2 = self.transform_point_cloud_service(resp.point_cloud, 'bounding_box', 'bin_K')


        

        return resp2.point_cloud


    def generate_plans(self, bounding_box, point_cloud):

        # Basically BFS to make a tree
        # maintain a queue of paths
        queue = []
        # push the first path into the queue
        tree = State(bounding_box, point_cloud)
        self.grasp_evaluator.move_arms_to_side()
        self.grasp_evaluator.set_start_pose()
        grasps = self.grasp_evaluator.get_grasps(point_cloud)
        tree.add_grasps(grasps)
        queue.append([tree])
        count = 1
        while queue:
            # get the first path from the queue
            path = queue.pop(0)
            # get the last node from the path
            node = path[-1]
            # path found

            # rospy.loginfo("Actions: {}".format(self.actions))

            if not node.tipped:

                for action in self.actions:
                    x, y, yaw, x_dim, y_dim, z_dim = self.regressor.predict(action, node.bounding_box)
                    x_diff = x - node.bounding_box.pose.pose.position.x
                    y_diff = y - node.bounding_box.pose.pose.position.y
                    quaternion = (
                        node.bounding_box.pose.pose.orientation.x,
                        node.bounding_box.pose.pose.orientation.y,
                        node.bounding_box.pose.pose.orientation.z,
                        node.bounding_box.pose.pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    yaw_old = euler[2]
                    yaw_diff = yaw - yaw_old

                    new_bounding_box = copy.deepcopy(node.bounding_box)
                    new_bounding_box.pose.pose.position.x = x
                    new_bounding_box.pose.pose.position.y = y
                    new_bounding_box.dimensions.x = x_dim
                    new_bounding_box.dimensions.y = y_dim
                    new_bounding_box.dimensions.z = z_dim

                    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                    #type(pose) = geometry_msgs.msg.Pose
                    new_bounding_box.pose.pose.orientation.x = q[0]
                    new_bounding_box.pose.pose.orientation.y = q[1]
                    new_bounding_box.pose.pose.orientation.z = q[2]
                    new_bounding_box.pose.pose.orientation.w = q[3]

                    new_point_cloud = self.transform_point_cloud(node.point_cloud, node.bounding_box.pose, x, y, yaw)
                    points = pc2.read_points(new_point_cloud, skip_nans=True)
                    point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]

                    publish_cluster(self.markers, point_list, 'bin_K', 'experiment', 0)

                    rospy.loginfo("Type: {}".format(type(new_point_cloud)))

                    temp = State(copy.deepcopy(new_bounding_box), copy.deepcopy(new_point_cloud))

                    temp.add_actions(action, node.actions)

                    rospy.loginfo("Actions: {}".format(temp.actions))


                    temp_grasps = self.grasp_evaluator.get_grasps(new_point_cloud)

                    if math.fabs(z_dim - node.bounding_box.dimensions.z) > 0.025:
                        temp.tipped = True
                        rospy.loginfo("Item Tipped!")

                    if not temp.tipped:
                        temp.add_grasps(temp_grasps)

                    node.add_child(copy.deepcopy(temp))

                    count +=1
                    rospy.loginfo("Number of nodes: {}".format(count))
            else:
                count += 10

            if count > self.max_nodes:
                break

            # enumerate all adjacent nodes, construct a new path and push it into the queue
            for child in node.get_children():
                
                new_path = list(path)
                new_path.append(child)
                queue.append(new_path)

        rospy.loginfo("Number of nodes: {}".format(count))
        return tree

    def choose_action(self, state_tree):
        # Basically BFS to find first graspable
        # maintain a queue of paths
        if state_tree.grasps:
            return state_tree.grasps
        queue = []
        # push the first path into the queue
        queue.append([state_tree])
        while queue:
            # get the first path from the queue
            path = queue.pop(0)
            # get the last node from the path
            node = path[-1]
            # path found
            if node.grasps:
                return node.actions[0]

            # enumerate all adjacent nodes, construct a new path and push it into the queue
            for child in node.get_children():
                new_path = list(path)
                new_path.append(child)
                queue.append(new_path)
        return None
    def add_shelf(self):
        shelf_odom = PoseStamped()
        shelf_odom.header.frame_id = 'odom_combined'
        shelf_odom.pose.position.x = 1.19 #1.14
        shelf_odom.pose.position.y = 0.0 #0.20
        shelf_odom.pose.position.z = 0.0
        shelf_odom.pose.orientation.x = 0.0
        shelf_odom.pose.orientation.y = 0.0
        shelf_odom.pose.orientation.z = 0.0
        shelf_odom.pose.orientation.w = 1.0

        # Publish static transform for shelf
        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self.set_static_tf.wait_for_service()
        self.set_static_tf(transform)

        # Publish static transform for bin
        shelf_depth = 0.87
        bottom_row_z = 0.82 #0.806
        center_column_y = 0.0

        bin_translation = Vector3(
            x=-shelf_depth / 2.,
            y=center_column_y,
            z=bottom_row_z)

        transform = TransformStamped(
            header=Header(frame_id='shelf',
                          stamp=rospy.Time.now(), ),
            transform=Transform(translation=bin_translation,
                                rotation=Quaternion(w=1,
                                                    x=0,
                                                    y=0,
                                                    z=0), ),
            child_frame_id='bin_K', )
        self.set_static_tf.wait_for_service()
        self.set_static_tf(transform)

        scene = moveit_commander.PlanningSceneInterface()

        for i in range(10):
            scene.remove_world_object("bbox")
            scene.remove_world_object("shelf1")
            scene.remove_world_object("shelf4")
            scene.remove_world_object("shelf2")
            scene.remove_world_object("shelf3")

        wall_pose1 = PoseStamped()
        wall_pose1.header.frame_id = "base_footprint"
        wall_pose1.pose.position.x = 0.97
        wall_pose1.pose.position.y = 0.025 - 0.2
        wall_pose1.pose.position.z = 0.97

        wall_pose2 = PoseStamped()
        wall_pose2.header.frame_id = "base_footprint"
        wall_pose2.pose.position.x = 0.97
        wall_pose2.pose.position.y = 0.385 - 0.2
        wall_pose2.pose.position.z = 0.97

        wall_pose3 = PoseStamped()
        wall_pose3.header.frame_id = "base_footprint"
        wall_pose3.pose.position.x = 0.97
        wall_pose3.pose.position.y = 0.20 - 0.2
        wall_pose3.pose.position.z = 1.16
        
        wall_pose4 = PoseStamped()
        wall_pose4.header.frame_id = "base_footprint"
        wall_pose4.pose.position.x = 0.97
        wall_pose4.pose.position.y = 0.20 - 0.2
        wall_pose4.pose.position.z = 0.79

        rate = rospy.Rate(1)
        for i in range(10):
            #scene.add_box("table", table_pose, (0.38, 0.38, 0.78))
            scene.add_box("shelf1", wall_pose1, (0.38, 0.015, 0.38 ))
            scene.add_box("shelf2", wall_pose2, (0.38, 0.015, 0.38 ))
            scene.add_box("shelf3", wall_pose3, (0.38, 0.38, 0.015 ))
            scene.add_box("shelf4", wall_pose4, (0.38,0.38,0.015))
            rospy.sleep(1)
            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('grasp_evaluator_node')



    rospack = rospkg.RosPack()
    path = str(rospack.get_path('pr2_grasp_evaluator')) + '/training_data' 
    planner = PushPullPlanner()
    planner.add_shelf()

    planner.train(path)

    file_list = [str(rospack.get_path('pr2_grasp_evaluator')) + '/test_data/test_1.bag']

    for file_name in file_list:
        if file_name[-13:] == 'evaluated.bag':
            continue
        rospy.loginfo("Current file: " + file_name)
        bag = rosbag.Bag(file_name)
        for topic, trial_msg, t in bag.read_messages():
            bounding_box = trial_msg.before.boundingbox
            point_cloud = trial_msg.before.pointcloud2

        bag.close()

    results = planner.generate_plans(bounding_box, point_cloud)

    rospy.loginfo("Results: {}".format(results))

    action = planner.choose_action(results)

    rospy.loginfo("Chose action".format(action))



    