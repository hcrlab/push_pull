import actionlib
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from copy import deepcopy
import datetime
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point, PointStamped
from joblib import Parallel, delayed 
import json
import math
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

import outcomes
from pr2_pick_manipulation.srv import GetPose, MoveArm, SetGrippers, MoveArmIkRequest
from pr2_pick_perception.msg import Box
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest, PlanarPrincipalComponentsRequest, \
    DeleteStaticTransformRequest, BoxPointsResponse #, Cluster

from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox2, FindClusterBoundingBox2Request
import numpy as np

def dummy(idx, box, num_points, transformed_point):
        if (transformed_point.point.x >= box.min_x and
                transformed_point.point.x < box.max_x and
                transformed_point.point.y >= box.min_y and
                transformed_point.point.y < box.max_y and
                transformed_point.point.z >= box.min_z and
                transformed_point.point.z < box.max_z):
                num_points[idx] += 1


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
            output_keys=['re_sense_attempt']
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

    def downsample_cluster(self, cluster):

        points = pc2.read_points(
            cluster.pointcloud,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )

        downsampled_point_list = []
        point_list = []

        count = 0

        #point_temp = deepcopy(points)
        #length = len(list(point_temp))

        #rospy.loginfo("Points in cluster: " + str(length))

        #if length > 500:
        #    a = 2
        #else:
        #    a = 1

        a = 2

        rospy.loginfo("Frame for cluster: " + str(cluster.header.frame_id))

        for x, y, z in points:
            #rospy.loginfo("Not adding point")
            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            point.header.frame_id = cluster.header.frame_id
            if (count % a) == 0:
                downsampled_point_list.append(point)
            count += 1
            point_list.append(point)

        if len(downsampled_point_list) < 250:
            return point_list
        else:
            return downsampled_point_list

    def find_points_in_box(self, request):
        ''' Returns number of points within bounding box specified by 
            request. ''' 

        #rospy.loginfo("In finding points!")       
        points = pc2.read_points(
            request.cluster.pointcloud,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )

        num_points = [0] * len(request.boxes)

        start = datetime.datetime.now()

        for x, y, z in points:

            # Transform point into frame of bounding box
            point = PointStamped(
                point=Point(x=x, y=y, z=z),
                header=Header(
                    frame_id=request.cluster.header.frame_id,
                    stamp=rospy.Time(0),
                )
            )

            #self._tf_listener.waitForTransform(request.cluster.header.frame_id, 
            #                                request.frame_id, rospy.Time(0),
            #                                rospy.Duration(10.0))

            transformed_point = self._tf_listener.transformPoint(request.frame_id,
                                                                point)
            
            for (idx, box) in enumerate(request.boxes):
                if (transformed_point.point.x >= box.min_x and
                    transformed_point.point.x < box.max_x and
                    transformed_point.point.y >= box.min_y and
                    transformed_point.point.y < box.max_y and
                    transformed_point.point.z >= box.min_z and
                    transformed_point.point.z < box.max_z):
                    num_points[idx] += 1
            
 
            #mylambaxfn = lambda idx,box:dummy(idx, box, num_points)
            #Parallel(n_jobs = 2)(delayed(dummy)(idx, box, num_points, transformed_point) for (idx, box) in enumerate(request.boxes))
        end = datetime.datetime.now()

        self.timer += ((end - start).total_seconds()) 

        #rospy.loginfo("Leaving finding points!")       
        return BoxPointsResponse(num_points=num_points)

        
    def find_points_in_box_downsampled(self, point_list, boxes, frame_id):
        ''' Returns number of points within bounding box specified by 
            request. '''        
        #points = pc2.read_points(
        #    request.cluster.pointcloud,
        #    field_names=['x', 'y', 'z'],
        #    skip_nans=True,
        #)

        start = datetime.datetime.now()

        num_points = [0] * len(boxes)
        for point in point_list:

            # Transform point into frame of bounding box
            #point = PointStamped(
            #    point=Point(x=x, y=y, z=z),
            #    header=Header(
            #        frame_id=request.cluster.header.frame_id,
            #        stamp=rospy.Time(0),
            #    )
            #)

            #self._tf_listener.waitForTransform(request.cluster.header.frame_id, 
            #                                request.frame_id, rospy.Time(0),
            #                                rospy.Duration(10.0))


            transformed_point = self._tf_listener.transformPoint(frame_id,
                                                                point)

            for (idx, box) in enumerate(boxes):
                if (transformed_point.point.x >= box.min_x and
                    transformed_point.point.x <= box.max_x and
                    transformed_point.point.y >= box.min_y and
                    transformed_point.point.y <= box.max_y and
                    transformed_point.point.z >= box.min_z and
                    transformed_point.point.z <= box.max_z):
                    num_points[idx] += 1
        end = datetime.datetime.now()

        self.timer += ((end - start).total_seconds()) 


        return num_points

       

    def get_bounding_box_corners(self, item_descriptor):
        """ 
        Get the corners of the bounding box
        Also returns the closest corner to the robot and
        the points on the middle of the faces closest to the robot
        These things aren't super related 
        but because calculating them required the same tf frames, here they are!
        """
        bounding_box = item_descriptor.planar_bounding_box


        (trans, rot) = self._tf_listener.lookupTransform("base_footprint", "head_mount_link", rospy.Time(0))
        self.loginfo("Tranform: {}, {}".format(trans, rot))

        quaternion = (
            rot[0],
            rot[1],
            rot[2],
            rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = 0
        pitch = 0
        yaw = euler[2] 

        pose = PoseStamped()

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]

        
        
        # Transform object pose into new frame
        bbox_in_head_yaw = self._tf_listener.transformPose('head_yaw', bounding_box.pose)

        corners = []

        corner_1 = PointStamped()
        corner_1.header.frame_id = 'bounding_box'
        corner_1.point.x = bounding_box.dimensions.x/2
        corner_1.point.y = bounding_box.dimensions.y/2

        corner_2 = PointStamped()
        corner_2.header.frame_id = 'bounding_box'
        corner_2.point.x = bounding_box.dimensions.x/2
        corner_2.point.y = -bounding_box.dimensions.y/2

        corner_3 = PointStamped()
        corner_3.header.frame_id = 'bounding_box'
        corner_3.point.x = -bounding_box.dimensions.x/2
        corner_3.point.y = bounding_box.dimensions.y/2

        corner_4 = PointStamped()
        corner_4.header.frame_id = 'bounding_box'
        corner_4.point.x = -bounding_box.dimensions.x/2
        corner_4.point.y = -bounding_box.dimensions.y/2

        corners = [corner_1, corner_2, corner_3, corner_4]

        self.loginfo("Corners: {}".format(corners))

        transformed_corners = []
        
        self._tf_listener.waitForTransform("head_yaw", 'bounding_box', rospy.Time(0), rospy.Duration(10.0))
        for corner in corners:
            new_corner = self._tf_listener.transformPoint("head_yaw", corner)
            transformed_corners.append(new_corner)

        min_dist = 1000
        closest_corner = None
        closest_corner_idx = 1000
        for (idx, corner) in enumerate(transformed_corners):
            dist = math.sqrt(pow(pose.pose.position.x - corner.point.x, 2) +
                         pow(pose.pose.position.y - corner.point.y, 2))
            if dist < min_dist:
                min_dist = dist
                closest_corner = corner
                closest_corner_idx = idx


        # Find corner with same x value
        closest_corner_in_head_yaw = corners[closest_corner_idx]
        face_1_corners = [closest_corner_in_head_yaw]
        for corner in corners:
            if (not corner.point.y == closest_corner_in_head_yaw.point.y) and \
                (corner.point.x == closest_corner_in_head_yaw.point.x):
                face_1_corners.append(corner)

        # Get Avg y value between them 
        avg_y = (face_1_corners[0].point.y + face_1_corners[1].point.y)/2


        face_1_grasp_point = PointStamped()
        face_1_grasp_point.header.frame_id = face_1_corners[0].header.frame_id
        face_1_grasp_point.point.x = face_1_corners[0].point.x
        face_1_grasp_point.point.z = face_1_corners[0].point.z
        face_1_grasp_point.point.y = avg_y

        # Put them back into the frame of the bounding box
        face_1_grasp_point_in_bbox_frame = self._tf_listener.transformPoint(
                                bounding_box.pose.header.frame_id, face_1_grasp_point)
        face_1_grasp_point_in_bbox_frame.point.z = bounding_box.dimensions.z

        # Find corner with same y value
        #closest_corner_in_head_yaw = corners[closest_corner_idx]
        face_2_corners = [closest_corner_in_head_yaw]
        self.loginfo("Corners: {}".format(corners))
        self.loginfo("Closest corner in head yaw: {}".format(closest_corner_in_head_yaw))
        for corner in corners:
            if (not corner.point.x == closest_corner_in_head_yaw.point.x) and \
                (corner.point.y == closest_corner_in_head_yaw.point.y):
                face_2_corners.append(corner)

        # Get Avg x value between them 
        avg_x = (face_2_corners[0].point.x + face_2_corners[1].point.x)/2


        face_2_grasp_point = PointStamped()
        face_2_grasp_point.header.frame_id = face_2_corners[0].header.frame_id
        face_2_grasp_point.point.x = face_2_corners[0].point.x
        face_2_grasp_point.point.y = face_2_corners[0].point.y
        face_2_grasp_point.point.x = avg_x

        # Put them back into the frame of the bounding box
        face_2_grasp_point_in_bbox_frame = self._tf_listener.transformPoint(
                        bounding_box.pose.header.frame_id, face_2_grasp_point)
        face_2_grasp_point_in_bbox_frame.point.z = bounding_box.dimensions.z

        closest_corner = self._tf_listener.transformPoint(
                        bounding_box.pose.header.frame_id, closest_corner_in_head_yaw)
        
        
        return corners, closest_corner, face_1_grasp_point_in_bbox_frame,\
                face_2_grasp_point_in_bbox_frame


    def add_shelf_mesh_to_scene(self, scene):
        q = tf.transformations.quaternion_from_euler(1.57,0,1.57)
        shelf_pose = PoseStamped(
            header=Header(frame_id='/shelf'),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            ),
        )
        rospack = rospkg.RosPack()
        path = rospack.get_path('pr2_pick_contest')
        shelf_mesh = path + '/config/kiva_pod/meshes/pod_lowres.stl'
        scene.add_mesh('shelf', shelf_pose, shelf_mesh)

    def log_pose_info(self, pose):
        position = pose.position
        self.loginfo(
            'pose x: {}, y: {}, z: {}'
            .format(position.x, position.y, position.z)
        )
        orientation = pose.orientation
        self.loginfo(
            'orientation x: {}, y: {}, z: {}'
            .format(orientation.x, orientation.y, orientation.z, orientation.w)
        )

    def grasp_msg_to_poses(self, grasp_msgs, object_pose, rotate=False):

        """
        Take list of Grasp msgs from moveit_simple_grasps service and make 
        dict of grasp and pre-grasp poses
        """

        grasping_pairs = []

        # get rid of every second message because they're just repeats in different frames
        grasp_msgs = grasp_msgs[::2]

        for grasp in grasp_msgs:
            if not grasp.grasp_quality >= self.min_grasp_quality:
                continue
 
            grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                grasp.grasp_pose)

            # Check if grasp is from behind or too high or low
            if grasp_pose.pose.position.x > object_pose.pose.position.x or \
                grasp_pose.pose.position.z > (self.shelf_bottom_height + 0.24) or \
                grasp_pose.pose.position.z < self.shelf_bottom_height:
                continue

            transform = TransformStamped()
            transform.header.frame_id = 'base_footprint'
            transform.header.stamp = rospy.Time.now()
            transform.transform.translation = grasp_pose.pose.position
            transform.transform.rotation = grasp_pose.pose.orientation
            transform.child_frame_id = 'grasp'
            self._set_static_tf.wait_for_service()
            self._set_static_tf(transform)
            rospy.sleep(0.25)

            
            pre_grasp_pose = PoseStamped()
            pre_grasp_pose.header.stamp = rospy.Time(0)
            pre_grasp_pose.header.frame_id = "grasp"
            pre_grasp_pose.pose.position.x = -1 * self.pre_grasp_offset
            pre_grasp_pose.pose.orientation.w = 1

            base_footprint_pre_grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_pose)

            self.loginfo(" ")
            self.loginfo(" ")

            grasp_dict = {}
            grasp_dict["pre_grasp"] = base_footprint_pre_grasp_pose
            grasp_dict["grasp"] = grasp_pose
            grasp_in_bounds = self.check_pose_within_bounds(grasp_pose, 
                                                                    self.shelf_bottom_height, self.shelf_height, 
                                                                    self.shelf_width, self.bin_id, 'base_footprint', False)

            
            if (not rotate) and (grasp_in_bounds):
                grasp_dict["id"] = self.grasp_num
                self.grasp_num += 1
                grasping_pairs.append(grasp_dict)
            
            pre_grasp_pose.header.stamp = rospy.Time(0)
            grasp_pose.header.stamp = rospy.Time(0)

            pre_grasp_pose = self.modify_grasp(pre_grasp_pose, -1.57, 0, 0, 0, 0, 0)

            base_footprint_pre_grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_pose)
            
            pose = PoseStamped()
            pose.header.frame_id = "grasp"
            pose.pose.orientation.w = 1

            pose = self.modify_grasp(pose, -1.57, 0, 0, 0, 0, 0)

            grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pose)
            grasp_dict = {}
            grasp_dict["pre_grasp"] = base_footprint_pre_grasp_pose

            grasp_dict["grasp"] = grasp_pose
            grasp_dict["id"] = self.grasp_num
            self.grasp_num += 1
            grasp_in_bounds = self.check_pose_within_bounds(grasp_pose, 
                                                                    self.shelf_bottom_height, self.shelf_height, 
                                                                    self.shelf_width, self.bin_id, 'base_footprint', False)


            if rotate and grasp_in_bounds:
                grasping_pairs.append(grasp_dict)
            
        return grasping_pairs


    def modify_grasp(self, pose, r, p, y, x, y_pos, z):
        """
        Apply rotation and translation to pose and return a copy
        """
        new_pose = PoseStamped()
        new_pose.header.frame_id = pose.header.frame_id
        new_pose.header.stamp = rospy.Time(0)
        new_pose.pose.position.x = pose.pose.position.x + x
        new_pose.pose.position.y = pose.pose.position.y + y_pos 
        new_pose.pose.position.z = pose.pose.position.z + z
        new_pose.pose.orientation.x = pose.pose.orientation.x
        new_pose.pose.orientation.y = pose.pose.orientation.y
        new_pose.pose.orientation.z = pose.pose.orientation.z
        new_pose.pose.orientation.w = pose.pose.orientation.w

        quaternion = (
            new_pose.pose.orientation.x,
            new_pose.pose.orientation.y,
            new_pose.pose.orientation.z,
            new_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        roll = roll + r
        pitch = pitch + p
        yaw = yaw + y

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #type(pose) = geometry_msgs.msg.Pose
        new_pose.pose.orientation.x = quaternion[0]
        new_pose.pose.orientation.y = quaternion[1]
        new_pose.pose.orientation.z = quaternion[2]
        new_pose.pose.orientation.w = quaternion[3]

        return new_pose

    def move_pose_within_bounds(self, pose, shelf_bottom_height, shelf_height, 
                                    shelf_width, bin_id, frame, rotated):
        """
        Check if a pose is within the width and height of the bin
        i.e. That it will not hit the shelf
        If it would hit the shelf, move it so that it would not
        """

        pose_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                pose)

        # pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
        #                                                         pose)

        # Check if within bin_width
        if pose_in_bin_frame.pose.position.y > ((self.shelf_width/2) -  
                    (self.gripper_palm_width + self.bin_bound_left)/2):
            pose_in_bin_frame.pose.position.y = ((self.shelf_width/2) -  
                    (self.gripper_palm_width + self.bin_bound_left)/2)
        elif pose_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 +  
                    (self.gripper_palm_width + self.bin_bound_right)/2):
            pose_in_bin_frame.pose.position.y = ((-1*self.shelf_width/2 +  
                    (self.gripper_palm_width + self.bin_bound_right)/2))
       
        pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        pose_in_bin_frame)

        if rotated:
            gripper_offset = self.gripper_palm_width/2
        else:
            gripper_offset = self.half_gripper_height

        if ((pose_in_base_footprint.pose.position.z > 
                (self.shelf_bottom_height + 2*gripper_offset))
                and (pose_in_base_footprint.pose.position.z < 
                (self.shelf_bottom_height + self.shelf_height - (gripper_offset)))):
            pose_in_base_footprint.pose.position.z = pose_in_base_footprint.pose.position.z
        else:
            pose_in_base_footprint.pose.position.z = self.shelf_bottom_height + \
                                                        2*gripper_offset

        pose_in_frame = self._tf_listener.transformPose(frame,
                                                        pose_in_bin_frame)

        return pose_in_frame, pose_in_base_footprint

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
            self.loginfo("Pose >  y bounds")
            self.loginfo("Y pose: {}".format(pose_in_bin_frame.pose.position.y))
            self.loginfo("Bound: {}".format(((self.shelf_width/2) - 
                (self.gripper_palm_width + self.bin_bound_left)/2)))

        elif pose_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + 
            (self.gripper_palm_width + self.bin_bound_right)/2):
            in_bounds = False
            self.loginfo("Pose <  y bounds")
            self.loginfo("Y pose: {}".format(pose_in_bin_frame.pose.position.y))
            self.loginfo("Bound: {}".format(-1*self.shelf_width/2 + 
                (self.gripper_palm_width + self.bin_bound_right)/2))
        pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        pose_in_bin_frame)

        if rotated:
            gripper_offset = self.gripper_palm_width/2
        else:
            gripper_offset = self.half_gripper_height

        if ((pose_in_base_footprint.pose.position.z > 
                (self.shelf_bottom_height + 2*gripper_offset))
                and (pose_in_base_footprint.pose.position.z < 
                (self.shelf_bottom_height + self.shelf_height - (gripper_offset)))):
            pose_in_base_footprint.pose.position.z = pose_in_base_footprint.pose.position.z
        else:
            #if pose_in_bin_frame.pose.position.x > 0:
            in_bounds = False
            self.loginfo("Pose not within z bounds")
            self.loginfo("Z pose: {}".format(pose_in_base_footprint.pose.position.z))
            self.loginfo("Bound: {}".format((self.shelf_bottom_height + gripper_offset)))
            self.loginfo("Bound: {}".format((self.shelf_bottom_height + 
                                        self.shelf_height - gripper_offset)))

        pose_in_frame = self._tf_listener.transformPose(frame,
                                                        pose_in_bin_frame)

        return in_bounds



    def get_pca_aligned_grasps(self, bounding_box, axes_poses, bin_id):
        """
        Generate grasp and pre-grasp poses aligned with axes from PCA
        Makes a level, pitched and rolled grasp for each grasp point for each axis
        """

        # get target edge of bounding box 
        corners, closest_corner, grasp_point_on_face_1, grasp_point_on_face_2 = \
                            self.get_bounding_box_corners(self.target_descriptor)

        self.loginfo("Closest end of bounding box: {}".format(closest_corner))
       
        new_ends = []
        rejected_in_base_footprint = []
        for (idx, corner) in enumerate(corners):
            
            self._tf_listener.waitForTransform("base_footprint", corner.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
            new_end = self._tf_listener.transformPoint('base_footprint',
                                                                corner)
            new_ends.append(new_end)

            #new_rej = self._tf_listener.transformPoint('base_footprint',
            #                                                    rejected[idx])
            #rejected_in_base_footprint.append(new_rej)

            marker_pose = PoseStamped()
            marker_pose.header.frame_id = 'base_footprint'
            marker_pose.pose.position.x = new_end.point.x
            marker_pose.pose.position.y = new_end.point.y
            marker_pose.pose.position.z = new_end.point.z
            viz.publish_bounding_box(self._markers, marker_pose , 0.01, 0.01, 0.01,
            0.0, 1.0, 0.0, 0.5, idx + 20)
        
        if self._debug:
            raw_input('(Debug) Press enter to continue >')

        y_offset = self.gripper_palm_width/2

        # object_pose = bounding_box.pose

        # object_pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
        #                                                         object_pose)
        grasps = []

        centroid = PointStamped()
        centroid.header.frame_id = bounding_box.pose.header.frame_id
        centroid.point.x = bounding_box.pose.pose.position.x
        centroid.point.y = bounding_box.pose.pose.position.y
        centroid.point.z = bounding_box.pose.pose.position.z
            
        self.loginfo("Chosen end: {}".format(closest_corner))
        self.loginfo("Offset is: {}".format(y_offset))
        # Put wrist_roll_link at center and then move it back along x-axis
        for axis_pose in axes_poses:
            transform = TransformStamped()
            transform.header.frame_id = 'bin_' + str(bin_id)
            transform.header.stamp = rospy.Time.now()
            transform.transform.translation = axis_pose.pose.position
            transform.transform.rotation = axis_pose.pose.orientation
            transform.child_frame_id = 'object_axis'
            self._set_static_tf.wait_for_service()
            self._set_static_tf(transform)
            rospy.sleep(0.25)
            self._tf_listener.waitForTransform("object_axis", "bin_" + str(bin_id), rospy.Time(0), rospy.Duration(10.0))

            # Check if allowed to grasp this end of object
            # y_values = []
            # for (idx, corner) in enumerate(ends):
            #     # Transform into this frame
            #     point_1 = self._tf_listener.transformPoint('object_axis', corner)
            #     point_2 =  self._tf_listener.transformPoint('object_axis', rejected[idx])
               
            #     if not y_values:
            #         y_values.append(point_1.point.y)
            #     else:
            #         for value in y_values:
            #             if math.fabs(point_1.point.y - value) > 0.001:
            #                 y_values.append(point_1.point.y) 
            #     for value in y_values:
            #             if math.fabs(point_2.point.y - value) > 0.001:
            #                 y_values.append(point_2.point.y)
 
            #     #if point_1.point.y not in y_values:
            #     #    y_values.append(point_1)
            #     #if point_2.point.y not in y_values:
            #     #    y_values.append(point_2)

            # if (not self.grasp_wide_end) and (math.fabs(y_value[0] - y_value[1] > 0.10)):
            #     continue


            # Check if axes are in the right direction
            grasp_in_axis_frame = PoseStamped()
            grasp_in_axis_frame.header.frame_id = 'object_axis'
            grasp_in_axis_frame.pose.orientation.w = 1
            self._tf_listener.waitForTransform("base_footprint", grasp_in_axis_frame.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
            grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                        grasp_in_axis_frame)
            finger_tips_in_axis_frame = PoseStamped()
            finger_tips_in_axis_frame.header.frame_id = 'object_axis'
            finger_tips_in_axis_frame.pose.orientation.w = 1
            finger_tips_in_axis_frame.pose.position.x = grasp_in_axis_frame.pose.position.x + 0.15
            finger_tips_in_base_footprint =  self._tf_listener.transformPose('base_footprint',
                                                                        finger_tips_in_axis_frame) 

            if grasp_in_base_footprint.pose.position.x > (finger_tips_in_base_footprint.pose.position.x):
                # Axis is wrong way, reverse it

                # This method is not for this, but whatever

                new_axis_pose = self.modify_grasp(axis_pose, 0, 0, -3.14, 0, 0, 0)

                transform = TransformStamped()
                transform.header.frame_id = 'bin_' + str(bin_id)
                transform.header.stamp = rospy.Time.now()
                transform.transform.translation = new_axis_pose.pose.position
                transform.transform.rotation = new_axis_pose.pose.orientation
                transform.child_frame_id = 'object_axis'
                self._set_static_tf.wait_for_service()
                self._set_static_tf(transform)
                rospy.sleep(0.25)
                self._tf_listener.waitForTransform("object_axis", "bin_" + str(bin_id), rospy.Time(0), rospy.Duration(10.0))

            grasp_point_on_face_1_in_axis_frame = self._tf_listener.transformPoint('object_axis',
                                                                grasp_point_on_face_1)
            grasp_point_on_face_2_in_axis_frame = self._tf_listener.transformPoint('object_axis',
                                                                grasp_point_on_face_2)


            if (math.fabs(grasp_point_on_face_1_in_axis_frame.point.x) >  
                    math.fabs(grasp_point_on_face_2_in_axis_frame.point.x)):
                grasp_point_on_face = grasp_point_on_face_1
            else:
                grasp_point_on_face = grasp_point_on_face_2

            self._tf_listener.waitForTransform("object_axis", closest_corner.header.frame_id, rospy.Time(0), rospy.Duration(40.0))
            closest_corner_in_axis_frame = self._tf_listener.transformPoint('object_axis',
                                                                closest_corner)

            grasp_point_on_face_in_axis_frame = self._tf_listener.transformPoint('object_axis',
                                                                grasp_point_on_face)

            bbox_in_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                bounding_box.pose)

            bbox_in_bin_frame = self._tf_listener.transformPose("bin_" + str(bin_id),
                                                                bounding_box.pose)
            grasp_point_centroid = PointStamped()
            grasp_point_centroid.header.stamp = rospy.Time(0)
            grasp_point_centroid.header.frame_id = 'object_axis'
            grasp_point_centroid.point.z = bbox_in_axis_frame.pose.position.z
            
            grasp_point_centroid_align_edge = PointStamped()
            grasp_point_centroid_align_edge.header.stamp = rospy.Time(0)
            grasp_point_centroid_align_edge.header.frame_id = 'object_axis'
            if closest_corner_in_axis_frame.point.y > 0:
                grasp_point_centroid_align_edge.point.y = closest_corner_in_axis_frame.point.y - y_offset
            else:
                grasp_point_centroid_align_edge.point.y = closest_corner_in_axis_frame.point.y + y_offset
            grasp_point_centroid_align_edge.point.z = bbox_in_axis_frame.pose.position.z

            grasp_point_front_face = PointStamped()
            grasp_point_front_face.header.stamp = rospy.Time(0)
            grasp_point_front_face.header.frame_id = 'object_axis'
            grasp_point_front_face.point.x = grasp_point_on_face_in_axis_frame.point.x
            grasp_point_front_face.point.z = bbox_in_axis_frame.pose.position.z

            grasp_point_front_face_align_edge = PointStamped()
            grasp_point_front_face_align_edge.header.stamp = rospy.Time(0)
            grasp_point_front_face_align_edge.header.frame_id = 'object_axis'
            grasp_point_front_face_align_edge.point.x = grasp_point_on_face_in_axis_frame.point.x
            if closest_corner_in_axis_frame.point.y > 0:
                grasp_point_front_face_align_edge.point.y = closest_corner_in_axis_frame.point.y - y_offset
            else:
                grasp_point_front_face_align_edge.point.y = closest_corner_in_axis_frame.point.y + y_offset
            grasp_point_front_face_align_edge.point.z = bbox_in_axis_frame.pose.position.z

            axis_grasp_points = [grasp_point_centroid, grasp_point_centroid_align_edge, 
                                grasp_point_front_face, grasp_point_front_face_align_edge]

            

            # Grasp lowest point if not already at lowest point
            if self.grasp_multiple_heights and bbox_in_bin_frame.pose.position.z > (self.half_gripper_height + 0.03):
                bbox_in_bin_frame = self._tf_listener.transformPose('bin_' + str(self.bin_id), bounding_box.pose)
                bbox_in_bin_frame.pose.position.z = 2*self.half_gripper_height
                new_bbox_in_axis_frame = self._tf_listener.transformPose('object_axis', bbox_in_bin_frame)

                new_grasp_point_centroid = deepcopy(grasp_point_centroid)
                new_grasp_point_centroid.point.z = new_bbox_in_axis_frame.pose.position.z
                new_grasp_point_centroid_align_edge = deepcopy(grasp_point_centroid_align_edge)
                new_grasp_point_centroid_align_edge.point.z = new_bbox_in_axis_frame.pose.position.z
                new_grasp_point_front_face = deepcopy(grasp_point_front_face)
                new_grasp_point_front_face.point.z = new_bbox_in_axis_frame.pose.position.z
                new_grasp_point_front_face_align_edge = deepcopy(grasp_point_front_face_align_edge)
                new_grasp_point_front_face_align_edge.point.z = new_bbox_in_axis_frame.pose.position.z
                axis_grasp_point = axis_grasp_points + [new_grasp_point_centroid, new_grasp_point_centroid_align_edge, 
                                    new_grasp_point_front_face, new_grasp_point_front_face_align_edge]
            
            for (idx, grasp_point) in enumerate(axis_grasp_points):

       
                # Transform grasp point into axis frame 
                #grasp_point_in_axis_frame = self._tf_listener.transformPose('object_axis', grasp_point)

                marker_pose = PoseStamped()
                marker_pose.header.frame_id = grasp_point.header.frame_id
                marker_pose.pose.position.x = grasp_point.point.x
                marker_pose.pose.position.y = grasp_point.point.y
                marker_pose.pose.position.z = grasp_point.point.z
                viz.publish_bounding_box(self._markers, marker_pose , 0.01, 0.01, 0.01,
                0.0, 0.6, 0.8, 0.5, 1500)
     

                # Transform bounding box info into frame of PCA axes
                bbox_pose_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                    bounding_box.pose)
                object_pose = self._tf_listener.transformPose('base_footprint',
                                                                    bounding_box.pose)
                grasp_in_axis_frame = PoseStamped()
                grasp_in_axis_frame.header.stamp = rospy.Time(0)
                grasp_in_axis_frame.header.frame_id = 'object_axis'
                grasp_in_axis_frame.pose.orientation.w = 1
                grasp_in_axis_frame.pose.position.x = -1 * self.dist_to_palm + grasp_point.point.x
                grasp_in_axis_frame.pose.position.y = grasp_point.point.y
                grasp_in_axis_frame.pose.position.z = grasp_point.point.z


                # Transform into base_footprint
                grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                    grasp_in_axis_frame)

                # Transform grasp_point into base_footprint
                grasp_point_in_base_footprint = self._tf_listener.transformPoint('base_footprint',
                                                                    grasp_point)

                viz.publish_gripper(self._im_server, grasp_in_base_footprint, 'grasp_target')
                if self._debug:
                    raw_input('(Debug) Initial gripper estimate >')

                #grasp_in_axis_frame.pose.position.y += y_offsets[idx]
                self._tf_listener.waitForTransform('base_footprint', grasp_in_axis_frame.header.frame_id, rospy.Time(0), rospy.Duration(20.0))
                grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                grasp_in_axis_frame)                


                pre_grasp_in_axis_frame = PoseStamped()
                pre_grasp_in_axis_frame.header.frame_id = 'object_axis'
                pre_grasp_in_axis_frame.pose.position.y = grasp_in_axis_frame.pose.position.y
                pre_grasp_in_axis_frame.pose.position.x = grasp_in_axis_frame.pose.position.x - self.pre_grasp_offset
                pre_grasp_in_axis_frame.pose.position.z = grasp_in_axis_frame.pose.position.z
                pre_grasp_in_axis_frame.pose.orientation = grasp_in_axis_frame.pose.orientation
                pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_in_axis_frame)

                viz.publish_gripper(self._im_server, grasp_in_base_footprint, 'grasp_target')
                self.loginfo("Grasp point x: {}".format(grasp_point_in_base_footprint.point.x))
                self.loginfo("Gripper: {}".format(grasp_in_base_footprint.pose.position.x))

                if self._debug:
                    raw_input('(Debug) Aligned properly >') 

                grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                grasp_in_base_footprint)
                

                pre_grasp_in_axis_frame, pre_grasp_in_base_footprint = self.move_pose_within_bounds(pre_grasp_in_axis_frame,
                                                                    self.shelf_bottom_height, self.shelf_height,
                                                                    self.shelf_width, bin_id, 'object_axis', False)


                grasp_in_axis_frame, grasp_in_base_footprint = self.move_pose_within_bounds(grasp_in_axis_frame,
                                                                    self.shelf_bottom_height, self.shelf_height,
                                                                    self.shelf_width, bin_id, 'object_axis', False)                
                grasp_in_bounds = self.check_pose_within_bounds(grasp_in_axis_frame, 
                                                                    self.shelf_bottom_height, self.shelf_height, 
                                                                    self.shelf_width, bin_id, 'object_axis', False)
                if grasp_in_bounds:
                    grasp_dict = {}
                    grasp_dict["grasp"] = grasp_in_base_footprint
                    grasp_dict["pre_grasp"] = pre_grasp_in_base_footprint
                    grasp_dict["id"] = self.grasp_num
                    self.grasp_num += 1

                    grasps.append(grasp_dict)
                else:
                    self.loginfo("Central grasp not in bounds")

                # Make rotated version
                rolled_pre_grasp_in_axis_frame = self.modify_grasp(pre_grasp_in_axis_frame,
                                                                     -1.57, 0, 0, 0, 0, 0)

                
                self._tf_listener.waitForTransform('base_footprint', rolled_pre_grasp_in_axis_frame.header.frame_id, 
                                                    rospy.Time(0), rospy.Duration(20.0))
                rolled_pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                    rolled_pre_grasp_in_axis_frame)

                rolled_grasp_in_axis_frame = self.modify_grasp(grasp_in_axis_frame, 
                                                                     -1.57, 0, 0, 0, 0, 0)
                rolled_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                   rolled_grasp_in_axis_frame)

                grasp_in_bounds = self.check_pose_within_bounds(rolled_grasp_in_axis_frame, 
                                                                    self.shelf_bottom_height, self.shelf_height, 
                                                                    self.shelf_width, bin_id, 'object_axis', True)
                if not grasp_in_bounds:
                    self.loginfo("Rolled grasp not in bounds")

                    rolled_pre_grasp_in_axis_frame = self.modify_grasp(rolled_pre_grasp_in_axis_frame,
                                                                            0, 0, 0,-0.05, 0, 0.0)
                    rolled_pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                    rolled_pre_grasp_in_axis_frame)

                    rolled_pre_grasp_in_base_footprint = self.modify_grasp(rolled_pre_grasp_in_base_footprint,
                                                                            0, 0, 0, 0, 0, 0.09)
                    
                    rolled_grasp_in_base_footprint = self.modify_grasp(rolled_grasp_in_base_footprint,
                                                                            0, 0, 0, 0, 0, 0.10)
                    rolled_grasp_in_axis_frame = self._tf_listener.transformPose('object_axis', 
                                                                            rolled_grasp_in_base_footprint)
                    rolled_grasp_in_axis_frame = self.modify_grasp(rolled_grasp_in_axis_frame,
                                                                     0, 3.14/5, 0, 0, 0, 0)
                    rolled_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                   rolled_grasp_in_axis_frame)

                    grasp_in_bounds = self.check_pose_within_bounds(rolled_grasp_in_axis_frame,
                                                                    self.shelf_bottom_height, self.shelf_height,
                                                                    self.shelf_width, bin_id, 'object_axis', True)

                if grasp_in_bounds and ('rolled' in self.allowed_grasps):

                    self.loginfo("Adding rolled grasp")

                    grasp_dict = {}
                    grasp_dict["grasp"] = rolled_grasp_in_base_footprint
                    grasp_dict["pre_grasp"] = rolled_pre_grasp_in_base_footprint
                    grasp_dict["rolled"] = True
                    grasp_dict["id"] = self.grasp_num
                    self.grasp_num += 1

                    grasps.append(grasp_dict)
                else:
                    self.loginfo("No rolled grasp.")

                # Make pitched down
                pitched_grasp_in_axis_frame = self.modify_grasp(grasp_in_axis_frame,
                                                                 0, 3.14/5, 0, 0, 0, 0.0)
                pitched_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                    pitched_grasp_in_axis_frame)
                pitched_pre_grasp_in_base_footprint = self.modify_grasp(pre_grasp_in_base_footprint,
                                                                 0, 0, 0, 0, 0, 0.05)


                pitched_grasp_in_base_footprint = self.modify_grasp(pitched_grasp_in_base_footprint,
                                                                 0, 0, 0, 0, 0, 0.07)

                pitched_grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                    pitched_grasp_in_base_footprint)

                grasp_in_bounds = self.check_pose_within_bounds(pitched_grasp_in_axis_frame, 
                                                                    self.shelf_bottom_height, self.shelf_height, 
                                                                    self.shelf_width, bin_id, 'object_axis', False)
                if grasp_in_bounds and ('pitched' in self.allowed_grasps):
                    grasp_dict = {}
                    grasp_dict["grasp"] = pitched_grasp_in_base_footprint
                    grasp_dict["pre_grasp"] = pitched_pre_grasp_in_base_footprint
                    grasp_dict["pitched_down"] = True
                    grasp_dict["id"] = self.grasp_num
                    self.grasp_num += 1
                 
                    grasps.append(grasp_dict)
                else:
                    self.loginfo("No pitched grasp")

                #self._delete_static_tf.wait_for_service()
                #req = DeleteStaticTransformRequest()
                #req.parent_frame = "bin_" + str(bin_id)
                #req.child_frame = "object_axis"
                #self._delete_static_tf(req)

        return grasps


    def generate_grasps(self, bounding_box, bin_id):
        """
        Generate the poses for grasps and pre-grasps
        """

        object_pose = bounding_box.pose
        object_pose = self._tf_listener.transformPose('base_footprint',
                                                                object_pose)

        grasping_pairs = [] # list of pre_grasp/grasp dicts 

        self.shelf_bottom_height = self._shelf_bottom_heights[bin_id]

        # Pre-grasp: pose arm in front of bin

        pre_grasp_pose_target = PoseStamped()
        pre_grasp_pose_target.header.frame_id = 'base_footprint';

        # Make basic head-on grasp

        if not self.top_shelf:
            self.loginfo('Not in the top row')
            pre_grasp_pose_target.pose.orientation.w = 1
            pre_grasp_pose_target.pose.position.x = self.pre_grasp_x_distance 
            pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y

            # If pre-grasp is outside the width or height of bin, move it
            pre_grasp_in_bin_frame, pre_grasp_pose_target = self.move_pose_within_bounds(
                            pre_grasp_pose_target, self.shelf_bottom_height,  
                            self.shelf_height, self.shelf_width, bin_id, 
                            'bin_' + str(bin_id), False)

        else:
            # Special pose for top shelf
            self.loginfo('In top row')
            pre_grasp_pose_target.pose.orientation.x = 0.984
            pre_grasp_pose_target.pose.orientation.y = -0.013
            pre_grasp_pose_target.pose.orientation.z = 0.178
            pre_grasp_pose_target.pose.orientation.w = 0.028
            pre_grasp_pose_target.pose.position.x = 0.243 
            pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y
            pre_grasp_pose_target.pose.position.z = 1.508            
            pre_grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                pre_grasp_pose_target)
            # Check if within bin_width
            if pre_grasp_in_bin_frame.pose.position.y > (self.shelf_width/2  - 
                            (self.gripper_palm_width + self.bin_bound_left)/2):
                pre_grasp_in_bin_frame.pose.position.y = (self.shelf_width/2 - 
                            (self.gripper_palm_width + self.bin_bound_left)/2)
            elif pre_grasp_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + 
                            (self.gripper_palm_width + self.bin_bound_right)/2):
                pre_grasp_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + 
                            (self.gripper_palm_width + self.bin_bound_right)/2)
            pre_grasp_pose_target = self._tf_listener.transformPose('base_footprint',
                                                            pre_grasp_in_bin_frame)


        # Grasp: move gripper into bin to grasp object

        grasp_pose_target = PoseStamped()
        grasp_pose_target.header.frame_id = 'base_footprint';
        if not self.top_shelf:

            self.loginfo('Not grasping from top row')
            grasp_pose_target.pose.orientation.w = 1
            grasp_pose_target.pose.position.x = \
                object_pose.pose.position.x - self.dist_to_palm
            grasp_pose_target.pose.position.y = object_pose.pose.position.y

            grasp_in_bin_frame, grasp_pose_target = self.move_pose_within_bounds(
                            grasp_pose_target, self.shelf_bottom_height, self.shelf_height, 
                            self.shelf_width, bin_id, 'bin_' + str(bin_id), False)

        else:
            self.loginfo('Grasping from top row')
            grasp_pose_target.pose.orientation.x = 0.996
            grasp_pose_target.pose.orientation.y = -0.016
            grasp_pose_target.pose.orientation.z = 0.080
            grasp_pose_target.pose.orientation.w = 0.027
            grasp_pose_target.pose.position.x = 0.431
            grasp_pose_target.pose.position.y = object_pose.pose.position.y
            grasp_pose_target.pose.position.z = 1.57 
            grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                grasp_pose_target)
            # Check if within bin_width
            if grasp_in_bin_frame.pose.position.y > (self.shelf_width/2  - 
                            (self.gripper_palm_width + self.bin_bound_left)/2):
                grasp_in_bin_frame.pose.position.y = (self.shelf_width/2 - 
                            (self.gripper_palm_width + self.bin_bound_left)/2)
            elif grasp_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + 
                            (self.gripper_palm_width + self.bin_bound_right)/2):
                grasp_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + 
                            (self.gripper_palm_width + self.bin_bound_right)/2)
            grasp_pose_target = self._tf_listener.transformPose('base_footprint',
                                                            grasp_in_bin_frame)

        grasp_dict = {}
        grasp_dict["pre_grasp"] = pre_grasp_pose_target
        grasp_dict["grasp"] = grasp_pose_target
        grasp_dict["front_grasp"] = True
        grasp_dict["id"] = self.grasp_num
        self.grasp_num += 1
        grasping_pairs.append(grasp_dict)


        # If it's the top shelf, don't bother generating more grasps, 
        # the Pr2 can't reach them anyway
        if self.top_shelf:
            return grasping_pairs

        # Use moveit_simple_action_client to create fan of grasps around object
        grasp_action_client = actionlib.SimpleActionClient(
                '/moveit_simple_grasps_server/generate', GenerateGraspsAction)
        grasp_action_client.wait_for_server()

        # Do some math spatial acrobatics to get the pose in a 
        # moveit_simple_grasps-friendly orientation
        goal = GenerateGraspsGoal()
        pose = object_pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        pitch = pitch - 1.57

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        moveit_simple_grasps = []
        goal.pose.position.x = object_pose.pose.position.x
        goal.pose.position.y = object_pose.pose.position.y
        goal.pose.position.z = self.shelf_bottom_height + 2*self.half_gripper_height
        goal.pose.orientation = pose.orientation

        goal.width = 0.0
        
        goal_option_1 = GraspGeneratorOptions()
        goal_option_1.grasp_axis = goal_option_1.GRASP_AXIS_X
        goal_option_1.grasp_direction = goal_option_1.GRASP_DIRECTION_UP
        goal_option_1.grasp_rotation = goal_option_1.GRASP_ROTATION_FULL

        goal_option_2 = GraspGeneratorOptions()
        goal_option_2.grasp_axis = goal_option_2.GRASP_AXIS_X
        goal_option_2.grasp_direction = goal_option_2.GRASP_DIRECTION_DOWN
        goal_option_2.grasp_rotation = goal_option_2.GRASP_ROTATION_FULL

        goal.options = []
        goal.options.append(goal_option_1)
        goal.options.append(goal_option_2)
        grasp_action_client.send_goal(goal)
        grasp_action_client.wait_for_result()

        grasps_result = grasp_action_client.get_result()

        moveit_simple_grasps  =  moveit_simple_grasps + \
                                self.grasp_msg_to_poses(grasps_result.grasps,
                                                        object_pose, True)

        # Get pca aligned grasps
        self.loginfo("Getting PCA grasps")
        self._tf_listener.waitForTransform(object_pose.header.frame_id, "bin_" + str(bin_id), rospy.Time(0), rospy.Duration(10.0))
        object_pose_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                object_pose)

        self._get_planar_pca.wait_for_service()
        pca_resp = self._get_planar_pca(self._cluster)

        # Make poses out of the PCA axes
        first_pose = PoseStamped()
        first_pose.header.stamp = rospy.Time(0)
        first_pose.header.frame_id = "bin_" + str(bin_id)
        first_pose.pose.position = object_pose_in_bin_frame.pose.position
        first_pose.pose.orientation = pca_resp.first_orientation

        second_pose = PoseStamped()
        second_pose.header.stamp = rospy.Time(0)
        second_pose.header.frame_id = "bin_" + str(bin_id)
        second_pose.pose.position = object_pose_in_bin_frame.pose.position
        second_pose.pose.orientation = pca_resp.second_orientation

        axes_poses = [first_pose, second_pose]
        pca_grasps = self.get_pca_aligned_grasps(bounding_box, axes_poses, bin_id)

        grasping_pairs = grasping_pairs + moveit_simple_grasps + pca_grasps

        self.loginfo("Number of grasps: {}".format(len(grasping_pairs)))

        return grasping_pairs

    def get_grasp_intersections(self, grasp):
        """
        Check for intersections between gripper and point cloud
        Also check if there are points between gripper fingers
        """

        # Check if enough points will be in gripper
        self.loginfo("Checking grasp/cluster intersections") 
        y_offset = 0.005

        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        
        transform.transform.rotation = grasp['grasp'].pose.orientation
        transform.child_frame_id = 'grasp'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.25)

        #points_in_box_request = BoxPointsRequest()
        #points_in_box_request.frame_id = "grasp"
        #points_in_box_request.cluster = self._cluster

        boxes = []

        box_request = Box()
        box_request.min_x = self.dist_to_palm
        box_request.max_x = self.dist_to_fingertips
        box_request.min_y = -1 * self.gripper_palm_width/2 + y_offset
        box_request.max_y = self.gripper_palm_width/2 + y_offset
        box_request.min_z = -1 * self.gripper_finger_height/2
        box_request.max_z = 1 * self.gripper_finger_height/2
        boxes.append(box_request)

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'grasp'
        box_pose.pose.position.x = box_request.min_x + \
                    (box_request.max_x - box_request.min_x) / 2
        box_pose.pose.position.y = box_request.min_y + \
                    (box_request.max_y - box_request.min_y) / 2
        box_pose.pose.position.z = box_request.min_z + \
                    (box_request.max_z - box_request.min_z) / 2

        viz.publish_bounding_box(self._markers, box_pose, 
                    (box_request.max_x - box_request.min_x), 
                    (box_request.max_y - box_request.min_y), 
                    (box_request.max_z - box_request.min_z),
                    1.0, 0.0, 0.0, 0.5, 1)
        

        # Check for collisions with fingers
        # Left Finger
        l_finger_request = Box()
        l_finger_request.min_x = self.dist_to_palm - 0.02
        l_finger_request.max_x = self.dist_to_fingertips
        l_finger_request.min_y = -1 * self.gripper_palm_width/2 - 0.025 + y_offset
        l_finger_request.max_y = -1 * self.gripper_palm_width/2 - 0.005 + y_offset
        l_finger_request.min_z = -1 * self.gripper_finger_height/2
        l_finger_request.max_z = self.gripper_finger_height/2
        boxes.append(l_finger_request)
        
        l_finger_pose = PoseStamped()
        l_finger_pose.header.frame_id = 'grasp'
        l_finger_pose.pose.position.x = l_finger_request.min_x +\
                         (l_finger_request.max_x - l_finger_request.min_x) / 2
        l_finger_pose.pose.position.y = l_finger_request.min_y +\
                         (l_finger_request.max_y - l_finger_request.min_y) / 2
        l_finger_pose.pose.position.z = l_finger_request.min_z +\
                         (l_finger_request.max_z - l_finger_request.min_z) / 2

        viz.publish_bounding_box(self._markers, l_finger_pose, 
            (l_finger_request.max_x - l_finger_request.min_x), 
            (l_finger_request.max_y - l_finger_request.min_y), 
            (l_finger_request.max_z - l_finger_request.min_z),
            0.0, 0.0, 1.0, 0.5, 2)

        # Right Finger
        r_finger_request = Box()
        r_finger_request.min_x = self.dist_to_palm - 0.02
        r_finger_request.max_x = self.dist_to_fingertips
        r_finger_request.min_y = self.gripper_palm_width/2 + 0.005 + y_offset
        r_finger_request.max_y = self.gripper_palm_width/2 + 0.025 + y_offset
        r_finger_request.min_z = -1 * self.gripper_finger_height/2
        r_finger_request.max_z = self.gripper_finger_height/2
        boxes.append(r_finger_request)
        
        r_finger_pose = PoseStamped()
        r_finger_pose.header.frame_id = 'grasp'
        r_finger_pose.pose.position.x = r_finger_request.min_x +\
                         (r_finger_request.max_x - r_finger_request.min_x) / 2
        r_finger_pose.pose.position.y = r_finger_request.min_y +\
                         (r_finger_request.max_y - r_finger_request.min_y) / 2
        r_finger_pose.pose.position.z = r_finger_request.min_z +\
                         (r_finger_request.max_z - r_finger_request.min_z) / 2

        viz.publish_bounding_box(self._markers, r_finger_pose, 
            (r_finger_request.max_x - r_finger_request.min_x), 
            (r_finger_request.max_y - r_finger_request.min_y), 
            (r_finger_request.max_z - r_finger_request.min_z),
            0.0, 0.0, 1.0, 0.5, 3)

        # Check for collisions with palm
        palm_request = Box()
        palm_request.min_x = 0.05
        palm_request.max_x = self.dist_to_palm - 0.01
        palm_request.min_y = -1 * self.gripper_palm_width/2 + y_offset
        palm_request.max_y = self.gripper_palm_width/2 + y_offset
        palm_request.min_z = -1 * self.gripper_finger_height/2
        palm_request.max_z = self.gripper_finger_height/2
        boxes.append(palm_request)
        
        palm_pose = PoseStamped()
        palm_pose.header.frame_id = 'grasp'
        palm_pose.pose.position.x = palm_request.min_x +\
                         (palm_request.max_x - palm_request.min_x) / 2
        palm_pose.pose.position.y = palm_request.min_y +\
                         (palm_request.max_y - palm_request.min_y) / 2
        palm_pose.pose.position.z = palm_request.min_z +\
                         (palm_request.max_z - palm_request.min_z) / 2

        viz.publish_bounding_box(self._markers, palm_pose, 
            (palm_request.max_x - palm_request.min_x), 
            (palm_request.max_y - palm_request.min_y), 
            (palm_request.max_z - palm_request.min_z),
            0.0, 0.0, 1.0, 0.5, 4)

        #now = rospy.Time.now()
        #self.loginfo("Waiting for service: {}, {}".format(now.secs(), now.nsecs()))
        #self._get_points_in_box.wait_for_service()
        #now = rospy.Time.now()
        #self.loginfo("Calling service: {}, {}".format(now.secs(), now.nsecs()))
        #box_response = self._get_points_in_box(points_in_box_request)
        #now = rospy.Time.now()
        #self.loginfo("Received service response: {}, {}".format(now.secs(), now.nsecs()))

        # Moved point cloud checking out of service call
        num_points =  self.find_points_in_box_downsampled(self.downsampled_cluster, boxes, 'grasp')
        self.loginfo("Number of points inside gripper: {}".format(num_points[0]))
        self.loginfo("Number of points inside left finger: {}"
                        .format(num_points[1]))
        self.loginfo("Number of points inside right finger: {}"
                        .format(num_points[2]))
        self.loginfo("Number of points inside palm: {}"
                        .format(num_points[3]))

        grasp["finger_collision_points"] = num_points[1] + num_points[2]
        grasp["palm_collision_points"] = num_points[3]
        grasp["points_in_gripper"] = num_points[0]

        if grasp["points_in_gripper"] >= self.min_points_in_gripper and \
            grasp["finger_collision_points"] <= self.max_finger_collision_points and \
            grasp["palm_collision_points"] <= self.max_palm_collision_points:
            grasp["grasp_quality"] = (grasp["points_in_gripper"] -  
                                      grasp["finger_collision_points"] - 
                                      grasp["palm_collision_points"])  
            self.loginfo("Evaluated good grasp")
            self.loginfo("Grasp quality: " + str(grasp["grasp_quality"]))
            if 'front_grasp' in grasp:
                grasp["grasp_quality"] = grasp["grasp_quality"]  #self.max_grasp_quality
            if ('pitched_down' in grasp) and self.low_object:
                self.loginfo("Low object!")
                grasp["grasp_quality"] = 1.5 * grasp["grasp_quality"]  
        else:
            grasp["grasp_quality"] = 0.0
            self.loginfo("Evaluated bad grasp")
            self.loginfo("Grasp quality: " + str(grasp["grasp_quality"]))

        if self._debug:
            raw_input('(Debug) Press enter to continue >')

        return grasp

    def check_reachable(self, grasp):
        """
        Check if grasp/pre-grasp pair is reachable
        Uses Moveit planning to check pre-grasp
        Uses IK (no collision checking) for grasp
        """
        start = datetime.datetime.now()
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        transform.transform.rotation = grasp["grasp"].pose.orientation
        transform.child_frame_id = 'grasp'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.25)

        # Check pre-grasp feasibility 
        self.loginfo("Checking pre-grasp feasible")

        self._moveit_move_arm.wait_for_service()
        self.loginfo("_moveit_move_arm.wait_for_service() completed")
        success_pre_grasp = self._moveit_move_arm(grasp["pre_grasp"],
                                                  0.005, 0.005, 0, 'right_arm', 
                                                  True).success
        self.loginfo("Pre-grasp reachable: {}".format(success_pre_grasp))
        grasp["pre_grasp_reachable"] = success_pre_grasp

        # Check grasp IK
        self.loginfo("Checking grasp ik")

        # Could allow collisions with target item, but we don't right now
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

            #self._pubPlanningScene.publish(planning_scene_diff)
            #rospy.sleep(1.0)

        #success_grasp = self._moveit_move_arm(grasp["grasp"],
        #                                          0.01, 0.01, 0, 'right_arm', 
        #                                         True).success
        #self.loginfo("Grasp reachable: {}".format(success_grasp))
        #if success_grasp:
        #    grasp["grasp_reachable"] = True
        #else:
        #    grasp["grasp_reachable"] = False

        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "right_arm"
        ik_request.ik_request.timeout = self.ik_timeout
        ik_request.ik_request.pose_stamped = grasp["grasp"]
        ik_response = self._ik_client(ik_request)

        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            grasp["grasp_reachable"] = True
        else:
            grasp["grasp_reachable"] = False

        self.loginfo("Grasp reachable: {}".format(grasp["grasp_reachable"]))
        end = datetime.datetime.now()
        self.timer2 += (end - start).total_seconds() 
        return grasp

    def sort_grasps(self, grasps):
        """
        Sort grasps in descending order of quality
        """
        sorted_grasps = sorted(grasps, key=lambda k: k['grasp_quality'])
        sorted_grasps.reverse()
        return sorted_grasps


    def get_reachable_grasp(self, grasp):
        """
        Tries to change pre-grasp/grasp pairs that are not currently 
        reachable until they're reachable
        """
        start = datetime.datetime.now()
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        
        quaternion = (
            grasp["grasp"].pose.orientation.x,
            grasp["grasp"].pose.orientation.y,
            grasp["grasp"].pose.orientation.z,
            grasp["grasp"].pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        orientation = Quaternion()

        quaternion = tf.transformations.quaternion_from_euler(roll, 0, yaw)
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        
        transform.transform.rotation = orientation

        transform.child_frame_id = 'grasp_no_pitch'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.25)

        # If pre-grasp not reachable, try
        if not grasp["pre_grasp_reachable"]:
            pre_grasp_offsets = [
                self.pre_grasp_attempt_separation * i
                for i in range(self.pre_grasp_attempts)
            ]
            self.loginfo("Pre-grasp not reachable")
            reachable = False
            for (idx, offset) in enumerate(pre_grasp_offsets):
                # transform pre-grasp into r_wrist_roll_link frame
                transformed_pose = self._tf_listener.transformPose('grasp_no_pitch',
                                                            grasp["pre_grasp"])
                # move it back towards robot in x direction
                transformed_pose.pose.position.x = transformed_pose.pose.position.x - offset

                rotated = False
                if 'rolled' in grasp:
                    rotated = grasp['rolled']

                pre_grasp_in_bounds = True
                if not self.top_shelf:    
                    transformed_pose, pose_in_base_footprint = self.move_pose_within_bounds(transformed_pose, 
                                                            self.shelf_bottom_height, self.shelf_height, 
                                                            self.shelf_width, self.bin_id, 'base_footprint', rotated)
                
                self.loginfo("Checking pre-grasp feasibility")
               
                pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        transformed_pose)
                grasp["pre_grasp"] = pose_in_base_footprint
                viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')
                

                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(transformed_pose,
                                                  0.005, 0.005, 0, 'right_arm',
                                                  True).success
                grasp["pre_grasp_reachable"] = success_pre_grasp
                grasp["pre_grasp"] =  pose_in_base_footprint
                if success_pre_grasp:
                    self.loginfo("Found reachable pre-grasp.")
                    self.loginfo("Pre_grasp: {}".format(grasp["pre_grasp"]))
                    break
                    

        if not grasp["grasp_reachable"]:
            grasp_attempt_delta = \
                (self.dist_to_fingertips - self.dist_to_palm) / self.grasp_attempts
            grasp_attempt_offsets = [
                grasp_attempt_delta * i
                for i in range(self.grasp_attempts)
            ]
            self.loginfo("Grasp not reachable.")
            for (idx, offset) in enumerate(grasp_attempt_offsets):
                # transform grasp into r_wrist_roll_link frame
                transformed_pose = self._tf_listener.transformPose('grasp_no_pitch',
                                                            grasp["grasp"])

                # move it back towards robot in x
                transformed_pose.pose.position.x = transformed_pose.pose.position.x - offset

                rotated = False
                if 'rolled' in grasp:
                    rotated = grasp['rolled']

                grasp_in_bounds = True
                if not self.top_shelf:    
                    grasp_in_bounds = self.check_pose_within_bounds(transformed_pose, 
                                                            self.shelf_bottom_height, self.shelf_height, 
                                                            self.shelf_width, self.bin_id, 'base_footprint', rotated)
                if not grasp_in_bounds:
                    break

                
                #check ik
                ik_request = GetPositionIKRequest()
                ik_request.ik_request.group_name = "right_arm"
                ik_request.ik_request.timeout = self.ik_timeout
                ik_request.ik_request.pose_stamped = transformed_pose
                ik_response = self._ik_client(ik_request)
                
                viz.publish_gripper(self._im_server, transformed_pose, 'grasp_target')

                # Could have checked with Moveit and allowed collisions with target item
                # But doesn't seem to work very well 
                #self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
                #rospy.wait_for_service('/get_planning_scene', 10.0)
                #get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
                #request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
                #response = get_planning_scene(request)

                #acm = response.scene.allowed_collision_matrix
                #if not 'bbox' in acm.default_entry_names:
                #    # add button to allowed collision matrix 
                #    acm.default_entry_names += ['bbox']
                #    acm.default_entry_values += [True]

                #    planning_scene_diff = PlanningScene(
                #        is_diff=True,
                #        allowed_collision_matrix=acm)

                    #self._pubPlann
                    #ngScene.publish(planning_scene_diff)
                    #rospy.sleep(1.0)

                #success_grasp = self._moveit_move_arm(grasp["grasp"],
                #                                          0.01, 0.01, 0, 'right_arm',
                #                                         True).success

                if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                    #if success_grasp:
                    pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                            transformed_pose)
                    grasp["grasp"] = pose_in_base_footprint
                    grasp["grasp_reachable"] = True

                    # Check if shifted grasp still has object in gripper
                    grasp = self.get_grasp_intersections(grasp)
                    self.loginfo("Found reachable grasp.")
                    self.loginfo("Grasp: {}".format(grasp["grasp_reachable"]))
                    break

        end = datetime.datetime.now()
        self.timer2 += (end - start).total_seconds()      
        return grasp

    def filter_grasps(self, grasps):
        """
        Evaluate generated grasps
        Return feasible, high quality grasps
        """

        # High quality grasps where the gripper is in the right position
        good_grasps = []
        # Good grasps that are reachable
        reachable_good_grasps = []
        num_reachable = 0
        num_good = 0
        attempts = 0
        
        for (idx, grasp) in enumerate(grasps):
            self.loginfo("Now evaluating grasp: {}".format(idx))

            # For each generated grasp, 
            # check for intersections with the cluster point cloud

            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')

            grasp = self.get_grasp_intersections(grasp)

            self.loginfo("Evaluating Grasp {}".format(grasp['id']))

            # If the grasp is "almost good"
            # i.e. it has few points colliding with the fingers 
            # and lots of points between the fingers
            # but too many points in the hand
            # We then try to "back up" the grasp
            if grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                grasp["palm_collision_points"] > self.max_palm_collision_points:

                grasp_attempt_delta = 0.01
                grasp_attempts = 20
                grasp_attempt_offsets = [grasp_attempt_delta * i 
                                        for i in range(grasp_attempts)]

                self.loginfo("Good grasp but too many points in the palm.")
                for i in range(grasp_attempts):

                    transform = TransformStamped()
                    transform.header.frame_id = 'base_footprint'
                    transform.header.stamp = rospy.Time.now()
                    transform.transform.translation = grasp["grasp"].pose.position

                    quaternion = (
                        grasp["grasp"].pose.orientation.x,
                        grasp["grasp"].pose.orientation.y,
                        grasp["grasp"].pose.orientation.z,
                        grasp["grasp"].pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]

                    orientation = Quaternion()

                    quaternion = tf.transformations.quaternion_from_euler(roll, 0, yaw)
                    
                    orientation.x = quaternion[0]
                    orientation.y = quaternion[1]
                    orientation.z = quaternion[2]
                    orientation.w = quaternion[3]

                    transform.transform.rotation = orientation
                    transform.child_frame_id = 'grasp_no_pitch'
                    self._set_static_tf.wait_for_service()
                    self._set_static_tf(transform)
                    rospy.sleep(0.25)

                    transformed_pose = self._tf_listener.transformPose(
                                                                'grasp_no_pitch',
                                                                grasp["grasp"])
                    transformed_pre_grasp =  self._tf_listener.transformPose(
                                                                'grasp_no_pitch',
                                                                grasp["pre_grasp"])

                    self.loginfo("Backing off attempt {}.".format(idx))
                    # move it forward in x
                    transformed_pose.pose.position.x = \
                        transformed_pose.pose.position.x - grasp_attempt_delta

                    transformed_pre_grasp.pose.position.x = \
                        transformed_pre_grasp.pose.position.x - grasp_attempt_delta

                    pose_in_base_footprint = self._tf_listener.transformPose(
                                                                'base_footprint',
                                                                transformed_pose)

                    pre_grasp_in_base_footprint = self._tf_listener.transformPose(
                                                                'base_footprint',
                                                                transformed_pre_grasp)
                    grasp["grasp"] = pose_in_base_footprint

                    grasp["pre_grasp"] =  pre_grasp_in_base_footprint

                    viz.publish_gripper(self._im_server, grasp["grasp"], \
                                        'grasp_target')
                    viz.publish_gripper(self._im_server, grasp["pre_grasp"], 
                                        'pre_grasp_target')

                    self.loginfo("Pre-grasp: ")
                    bin_frame_pre_grasp = self._tf_listener.transformPose('bin_' + 
                                str(self.bin_id), grasp["pre_grasp"])
                    self.log_pose_info(bin_frame_pre_grasp.pose)
                    self.loginfo("Grasp: ")
                    bin_frame_grasp = self._tf_listener.transformPose('bin_' + 
                                str(self.bin_id), grasp["grasp"])
                    self.log_pose_info(bin_frame_grasp.pose)

                    rotated = False
                    if 'rolled' in grasp:
                        rotated = grasp['rolled']

                    # Check if grasp is within bounds 
                    # If not discard
                    # Move pre-grasp within bounds
                    grasp_in_bounds = True
                    if not self.top_shelf:
                        grasp_in_bounds = self.check_pose_within_bounds(
                                                                grasp['grasp'], 
                                                                self.shelf_bottom_height, 
                                                                self.shelf_height, 
                                                                self.shelf_width, 
                                                                self.bin_id, 
                                                                'base_footprint', 
                                                                rotated)
                        grasp["pre_grasp"], temp = self.move_pose_within_bounds(
                                                                grasp['pre_grasp'],
                                                                self.shelf_bottom_height, 
                                                                self.shelf_height,
                                                                self.shelf_width, 
                                                                self.bin_id, 
                                                                'base_footprint', 
                                                                rotated)
                        if not grasp_in_bounds:
                            self.loginfo("Grasp not in bounds")
                            break

                    # Check if grasp is good after shift
                    grasp = self.get_grasp_intersections(grasp)

                    if grasp["points_in_gripper"] >= self.min_points_in_gripper and \
                        grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                        grasp["palm_collision_points"] <= self.max_palm_collision_points:
                        self.loginfo("Good grasp.")
                        good_grasps.append(grasp)
                        break
                    if grasp["points_in_gripper"] == 0:
                        self.loginfo("Giving up on this grasp, no more points in gripper.")
                        break


            elif grasp["points_in_gripper"] >= self.min_points_in_gripper and \
                grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                grasp["palm_collision_points"] <= self.max_palm_collision_points:

                self.loginfo("Good grasp.")
                good_grasps.append(grasp)
        good_grasps = self.sort_grasps(good_grasps)
        num_reachable = 0
        # After finding high quality grasps, check if grasps are reachable
        for grasp in good_grasps:
            self.loginfo("Checking grasp: {}".format(grasp["id"]))
            grasp = self.check_reachable(grasp)
            if grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                self.loginfo("Grasp {} reachable".format(grasp['id']))
                reachable_good_grasps.append(grasp)
                num_reachable += 1
            else:
                self.loginfo("Grasp {} not reachable".format(grasp['id']))

            if num_reachable >= self.min_reachable:
                break

        # If no grasps are reachable then try to move them to be reachable
        if not reachable_good_grasps:
            for grasp in good_grasps:
                self.loginfo("Making grasp {} reachable".format(grasp["id"]))
                # Move and try to check IK
                self.loginfo("Trying to make grasp reachable.")
                grasp = self.get_reachable_grasp(grasp)
                if grasp["grasp_quality"] > self.min_grasp_quality and \
                    grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                    self.loginfo("Added grasp {} to reachable, good grasps"
                              .format(grasp['id']))
                    reachable_good_grasps.append(grasp)
                    num_reachable +=1
                if num_reachable >= self.min_reachable:
                    break


        return self.sort_grasps(reachable_good_grasps)

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
        req.side_step = side_step
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
    
        req = FindClusterBoundingBox2Request()
        req.cluster = cluster
        service_name = "find_cluster_bounding_box2"
        rospy.loginfo("waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox2)
        try:
            req = FindClusterBoundingBox2Request()
            req.cluster = cluster
	        res = serv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)  
            return 0
        if not res.error_code:
	       return (res.pose, res.box_dims)
        else:
            return (None, None)

        #call plan_point_cluster_grasp to get candidate grasps for a cluster
    def call_plan_point_cluster_grasp(self, cluster):
        
        req = GraspPlanningRequest()
        req.target.reference_frame_id = "/base_link"
        req.target.region.cloud = cluster
        req.arm_name = "right_arm"
        req.collision_support_surface_name = "table"
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
    def call_plan_point_cluster_grasp_action(self, cluster):
        
        goal = GraspPlanningGoal()
        goal.target.reference_frame_id = "/base_link"
        goal.target.region.cloud = cluster
        goal.arm_name = "right_arm"
        goal.collision_support_surface_name = "table"
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

    @handle_service_exceptions(outcomes.GRASP_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Started Grasp Planner")
        # self._cluster = userdata.target_cluster.pointcloud
        # #rospy.loginfo("CLUSTER:")
        # rospy.loginfo(type(self._cluster))
        # tf_broadcaster = tf.TransformBroadcaster()
        # tf_listener = tf.TransformListener()

        # #set params for planner (change to try different settings)
        # self.call_set_params(overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = False, pregrasp_just_outside_box = True, backoff_depth_steps = 1)

        # #(box_pose, box_dims) = self.call_find_cluster_bounding_box(self._cluster)

        # #viz.publish_bounding_box(self._markers, box_pose, 
        #            # (box_dims.x/2), 
        #            # (box_dims.y/2), 
        #            # (box_dims.z/2),
        #            # 1.0, 0.0, 0.0, 0.5, 1)

        # grasps = self.call_plan_point_cluster_grasp(self._cluster)
        # #grasps = self.call_plan_point_cluster_grasp_action(self._cluster.pointcloud)
        # #grasp_poses = [grasp.grasp_pose for grasp in grasps]
        # #rospy.loginfo(grasps)

        # return outcomes.GRASP_SUCCESS


        # start = datetime.datetime.now()

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

        self.downsampled_cluster = self.downsample_cluster(self._cluster)

        rospy.loginfo("Downsampled cluster size: " + str(len(self.downsampled_cluster)))

        if userdata.item_model.allow_finger_collisions:
            self.max_finger_collision_points = 1000

        self._tts.publish('Grasping {}'.format(userdata.item_model.speech_name))
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)

        # # Get the pose of the target item in the base frame
        # response = self._find_centroid(userdata.target_cluster)
        # item_point = response.centroid
        # item_pose = PoseStamped(
        #     header=Header(
        #         frame_id=item_point.header.frame_id,
        #         stamp=rospy.Time(0),
        #     ),
        #     pose=Pose(
        #         position=item_point.point,
        #         orientation=Quaternion(w=1, x=0, y=0, z=0),
        #     )
        # )

        self._tf_listener.waitForTransform("base_footprint", 
                userdata.target_descriptor.planar_bounding_box.pose.header.frame_id, 
                rospy.Time(0), rospy.Duration(10.0))

        base_frame_item_pose = self._tf_listener.transformPose('base_footprint',
                                    userdata.target_descriptor.planar_bounding_box.pose)

        self.loginfo(
            'Grasping item in bin {} from pose {}'
            .format(userdata.bin_id, base_frame_item_pose)
        )
        if userdata.debug:
            raw_input('(Debug) Press enter to continue >')

        # Add shelf and bounding box to planning scene
        planar_bounding_box = userdata.target_descriptor.planar_bounding_box
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object('bbox')
        scene.remove_world_object("shelf")

        self.add_shelf_mesh_to_scene(scene)

        # Weird loop that you have to do because Moveit it weird
        for i in range(10):
            self.loginfo("MoveIt looping hack, iteration %d" % i)
            scene.add_box("bbox", planar_bounding_box.pose, 
                        (planar_bounding_box.dimensions.x, 
                        planar_bounding_box.dimensions.y, 
                        planar_bounding_box.dimensions.z))
            rospy.sleep(0.1)

        # Make a frame that has the same yaw as head frame
        self.loginfo("About to wait for transform")
        (trans, rot) = self._tf_listener.lookupTransform("base_footprint", "head_mount_link", rospy.Time(0))
        self.loginfo("Tranform: {}, {}".format(trans, rot))

        quaternion = (
            rot[0],
            rot[1],
            rot[2],
            rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = 0
        pitch = 0
        yaw = euler[2] 

        pose = PoseStamped()

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]

        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = pose.pose.position
        transform.transform.rotation = pose.pose.orientation
        transform.child_frame_id = 'head_yaw'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.25)

        transform = TransformStamped()
        transform.header.frame_id = planar_bounding_box.pose.header.frame_id
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = planar_bounding_box.pose.pose.position
        transform.transform.rotation = planar_bounding_box.pose.pose.orientation
        transform.child_frame_id = 'bounding_box'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.25)


        viz.publish_bounding_box(self._markers,  planar_bounding_box.pose, planar_bounding_box.dimensions.x, planar_bounding_box.dimensions.y, planar_bounding_box.dimensions.z,
            1.0, 0.0, 0.0, 0.5, 25)
        grasp_gen_start = datetime.datetime.now()
        grasping_pairs = self.generate_grasps(planar_bounding_box, userdata.bin_id)
        grasp_gen_end = datetime.datetime.now()
        rospy.loginfo("Grasp gen: " + str((grasp_gen_end - grasp_gen_start).total_seconds()))
        filter_start = datetime.datetime.now()
        filtered_grasps = self.filter_grasps(grasping_pairs)
        filter_end = datetime.datetime.now()
        rospy.loginfo("Filter: " + str((filter_end - filter_start).total_seconds()))

        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "bin_" + str(self.bin_id)
        req.child_frame = "object_axis"
        self._delete_static_tf(req)

        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = "base_footprint"
        req.child_frame = "head_yaw"
        self._delete_static_tf(req)

        self._delete_static_tf.wait_for_service()
        req = DeleteStaticTransformRequest()
        req.parent_frame = planar_bounding_box.pose.header.frame_id
        req.child_frame = "bounding_box"
        self._delete_static_tf(req)
        end = datetime.datetime.now()
        self.timer1 += (end - start).total_seconds()
        rospy.loginfo("Total: " + str(self.timer1))
        rospy.loginfo("Intersection Function: " + str(self.timer))
        rospy.loginfo("IK/Moveit Functiona: " + str(self.timer2))


        if len(filtered_grasps) > 0:
            #success_grasp = self.execute_grasp(filtered_grasps, userdata.item_model)
            self.__tts.publish("The object is graspable.")
            self.loginfo("The object is graspable.")
            success_grasp = True
        else:
            #end = datetime.datetime.now()
            #self.timer1 += int((end - start).total_seconds())
            #rospy.loginfo("Total: " + str(self.timer1/1e6))
            #rospy.loginfo("Function: " + str(self.timer/1e6))
            self.loginfo('No good grasps found')
            self._tts.publish('No grasps found.')
            # if not userdata.re_grasp_attempt:
            #     userdata.re_sense_attempt = True
            #     return outcomes.GRASP_NONE
            # else:
            #     userdata.re_sense_attempt = False
            #     return outcomes.GRASP_FAILURE


        if self._debug:
                raw_input('(Debug) Press enter to continue >')

        if not success_grasp:
            self.loginfo('Grasping failed')
            self._tts.publish('Grasping failed. Giving up.')
            userdata.re_sense_attempt = False
            return outcomes.GRASP_FAILURE

        else:
            self.loginfo('Grasping succeeded')
            self._tts.publish('Grasping succeeded.')
            userdata.re_sense_attempt = False
            return outcomes.GRASP_SUCCESS
