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
from pr2_grasp_evaluator import Regressor
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
from pr2_pick_main import IdTable
from pr2_pick_main import publish_gripper, publish_bounding_box, publish_cluster
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class State(object):
    def __init__(self, bounding_box, point_cloud):
        self.children = []
        self.bounding_box = bounding_box
        self.point_cloud = point_cloud
        self.grasps = None
        self.actions = None

    def add_child(self, obj):
        self.children.append(obj)

    def get_children(self):
        return self.children

    def add_grasps(self, grasp):
        self.grasps = grasps
    def add_actions(self, action, prev_actions):
        self.actions = prev_actions.append(action)

class PushPullPlanner:
    def __init__(self):
        self.regressor = None
        self.max_depth = 3
        self.tf_listener = tf.TransformListener()
        self.set_static_tf = rospy.ServiceProxy('perception/set_static_transform',
                                            SetStaticTransform)
        self.delete_static_tf = rospy.ServiceProxy('perception/delete_static_transform',
                                             DeleteStaticTransform)
        self.transform_point_cloud_service = rospy.ServiceProxy('transform_point_cloud', TransformPointCloud)
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

    def train(self, path_to_data):
        self.regressor = Regressor(path_to_data)
        self.regressor.train()

    def get_depth(self, tree):
        count = 1
        current_node = tree
        while True:
            if current_node.get_children():
                count +=1
                current_node = current_node.get_children()[0]
            else:
                break
        return count

    def transform_point_cloud(self, point_cloud, pose, x, y, yaw):
        # Make tf frame at original bounding box origin
        transform = TransformStamped()
        transform.header.frame_id = 'bin_K'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = copy.deepcopy(pose.pose.position)
        transform.transform.rotation = copy.deepcopy(pose.pose.orientation)
        transform.child_frame_id = 'bounding_box'

        self.set_static_tf.wait_for_service()
    
        self.set_static_tf(transform)

        # tranform points into that frame
        self.transform_point_cloud(point_cloud, 'bounding_box')

        # Move tf frame to new bounding box origin
        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "bin_K"
        req.child_frame = "bounding_box"
        self._delete_static_tf(req)

        transform = TransformStamped()
        transform.header.frame_id = 'bin_K'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = pose.pose.position.z

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #type(pose) = geometry_msgs.msg.Pose
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        transform.child_frame_id = 'bounding_box'

        self.set_static_tf.wait_for_service()
    
        self.set_static_tf(transform)


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
        while queue:
            # get the first path from the queue
            path = queue.pop(0)
            # get the last node from the path
            node = path[-1]
            # path found

            for action in self.actions:
                x, y, yaw, x_dim, y_dim, z_dim = self.regressor.predict(action, node.bounding_box)
                x_diff = x - node.bounding_box.pose.pose.position.x
                y_diff = y - node.bounding_box.pose.pose.position.y
                quaternion = (
                    bounding_box.pose.pose.orientation.x,
                    bounding_box.pose.pose.orientation.y,
                    bounding_box.pose.pose.orientation.z,
                    bounding_box.pose.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                yaw_old = euler[2]
                yaw_diff = yaw - yaw_old

                new_bounding_box = copy.deepcopy(node.bounding_box)
                new_bounding_box.pose.position.x = x
                new_bounding_box.pose.position.y = y
                new_bounding_box.dimensions.x = x_dim
                new_bounding_box.dimensions.y = y_dim
                new_bounding_box.dimensions.z = z_dim

                q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                #type(pose) = geometry_msgs.msg.Pose
                new_bounding_box.pose.pose.orientation.x = quaternion[0]
                new_bounding_box.pose.pose.orientation.y = quaternion[1]
                new_bounding_box.pose.pose.orientation.z = quaternion[2]
                new_bounding_box.pose.pose.orientation.w = quaternion[3]

                new_point_cloud = self.transform_point_cloud(node.point_cloud, node.bounding_box.pose, x, y, yaw)

                temp = State(copy.deepcopy(new_bounding_box), copy.deepcopy(new_point_cloud))

                temp_grasps = self.grasp_evaluator.get_grasps(new_point_cloud)

                temp.add_actions(action, node.actions)

                temp.add_grasps(temp_grasps)

                node.add_child(copy.deepcopy(temp))

            if self.get_depth(tree) > self.max_depth:
                break

            # enumerate all adjacent nodes, construct a new path and push it into the queue
            for child in node.get_children():
                new_path = list(path)
                new_path.append(child)
                queue.append(new_path)



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



# graph is in adjacent list representation
graph = {
        '1': ['2', '3', '4'],
        '2': ['5', '6'],
        '5': ['9', '10'],
        '4': ['7', '8'],
        '7': ['11', '12']
        }

def get_depth(tree):
    count = 1
    current_node = tree
    while True:
        if current_node.get_children():
            count +=1
            current_node = current_node.get_children()[0]
        else:
            break
    return count





if __name__ == '__main__':

    # maintain a queue of paths
    queue = []
    # push the first path into the queue
    tree = Node()
    queue.append([tree])
    while queue:
        # get the first path from the queue
        path = queue.pop(0)
        # get the last node from the path
        node = path[-1]
        # path found

        for i in range(3):
            temp = Node()
            node.add_child(temp)

        if get_depth(tree) > 3:
            break

        # enumerate all adjacent nodes, construct a new path and push it into the queue
        for child in node.get_children():
            new_path = list(path)
            new_path.append(child)
            queue.append(new_path)

    print tree
    print "Length: " + str(len(tree.get_children()[2].get_children()))
    print "Length: " + str(len(tree.get_children()[2].get_children()[2].get_children()))
    