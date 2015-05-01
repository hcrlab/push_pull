import actionlib
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped
import json
import moveit_commander
from moveit_msgs.msg import Grasp
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal
from moveit_simple_grasps.msg import GraspGeneratorOptions
import os
import rospkg
import rospy
import smach
from std_msgs.msg import Header
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
import visualization as viz

import outcomes
from pr2_pick_manipulation.srv import GetPose, MoveArm, SetGrippers, MoveArmIkRequest
from pr2_pick_perception.msg import Box
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest

""" Temporary class for moving arm without collision checking until we have service """


class Grasp(smach.State):
    ''' Grasps an item in the bin. '''
    name = 'GRASP'

    # How many pre-grasp gripper positions to attempt
    pre_grasp_attempts = 3
    # Separation in meters between attempted pre-grasp positions
    pre_grasp_attempt_separation = 0.01
    # how many grasp gripper positions to attempt
    grasp_attempts = 20

    # desired distance from palm frame to object centroid
    pre_grasp_x_distance = 0.40

    # approximately half hand thickness
    half_gripper_height = 0.03
    # approximate distance from palm frame origin to palm surface
    dist_to_palm = 0.13
    # approximate distance from palm frame origin to fingertip with gripper closed
    dist_to_fingertips = 0.22

    # approx dist between fingers when gripper open
    gripper_palm_width = 0.08

    # approx height of pads of fingertips
    gripper_finger_height = 0.03

    pre_grasp_height = half_gripper_height + 0.02

    # minimum required grasp quality
    min_grasp_quality = 0.3

    # minimum number of points from cluster needed inside gripper for grasp
    min_points_in_gripper = 20

    # max number of points in cluster that can intersect with fingers
    max_finger_collision_points = 10

    max_palm_collision_points = 10

    shelf_height = None

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id', 'debug', 'target_cluster', 'current_target']
        )

        self._find_centroid = services['find_centroid']
        self._set_grippers = services['set_grippers']
        self._tuck_arms = services['tuck_arms']
        self._moveit_move_arm = services['moveit_move_arm']
        self._tts = services['tts']
        self._tf_listener = services['tf_listener']
        self._im_server = services['interactive_marker_server']
        self._set_static_tf = services['set_static_tf']
        self._markers = services['markers']
        self._move_arm_ik = services['move_arm_ik']

        self._fk_client = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self._ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._get_points_in_box = rospy.ServiceProxy('perception/get_points_in_box', BoxPoints)
        self._lookup_item = services['lookup_item']

        self._wait_for_transform_duration = rospy.Duration(5.0)

        self._cluster = None

        # Shelf heights

        self._shelf_height_a_c = 1.56
        self._shelf_height_d_f = 1.33
        self._shelf_height_g_i = 1.10
        self._shelf_height_j_l = 0.85

        self._shelf_heights = {
            'A': self._shelf_height_a_c,
            'B': self._shelf_height_a_c,
            'C': self._shelf_height_a_c,
            'D': self._shelf_height_d_f,
            'E': self._shelf_height_d_f,
            'F': self._shelf_height_d_f,
            'G': self._shelf_height_g_i,
            'H': self._shelf_height_g_i,
            'I': self._shelf_height_g_i,
            'J': self._shelf_height_j_l,
            'K': self._shelf_height_j_l,
            'L': self._shelf_height_j_l
        }

    def locate_hard_coded_items(self):
        '''
        Locate items in this shelf based on the hard-coded values in the json
        configuration file ignoring perception data. Intended to bypass
        perception.
        '''
        current_dir = os.path.dirname(__file__)
        relative_path = '../../config/milestone_1_fake_object_locations.json'
        file_path = os.path.join(current_dir, relative_path)

        with open(file_path) as config_file:
            object_data = json.load(config_file)

        item_pose = PoseStamped()

        for shelf_bin in object_data['work_order']:
            if (shelf_bin['bin'] == 'bin_' + str(userdata.bin_id) ):
                item_pose.pose.position.x = shelf_bin['pose']['x']
                item_pose.pose.position.y = shelf_bin['pose']['y']
                item_pose.pose.position.z = shelf_bin['pose']['z']
                break
        item_pose.header.frame_id = 'shelf'
        return [item_pose,]

    def add_shelf_mesh_to_scene(self, scene):
        q = tf.transformations.quaternion_from_euler(1.57,0,1.57)
        shelf_pose = PoseStamped(
            header=Header(frame_id='/shelf'),
            pose=Pose(
                position=Position(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            ),
        )
        rospack = rospkg.RosPack()
        path = rospack.get_path('pr2_pick_contest')
        shelf_mesh = path + '/config/kiva_pod/meshes/pod_lowres.stl'
        scene.add_mesh('shelf', shelf_pose, shelf_mesh)

    def log_pose_info(self, pose):
        position = pose.position
        rospy.loginfo(
            'pose x: {}, y: {}, z: {}'
            .format(position.x, position.y, position.z)
        )
        orientation = pose.orientation
        rospy.loginfo(
            'orientation x: {}, y: {}, z: {}'
            .format(orientation.x, orientation.y, orientation.z, orientation.w)
        )

    def grasp_msg_to_poses(self, grasp_msgs, object_pose, rotate=False):
        grasping_pairs = []

        # get rid of every second message because they're just repeats in different frames
        grasp_msgs = grasp_msgs[::2]

        for grasp in grasp_msgs:
            if not grasp.grasp_quality >= self.min_grasp_quality:
                continue
 
            # fk_request = GetPositionFKRequest()
            # fk_request.header.frame_id = "base_footprint"
            # fk_request.fk_link_names = ["r_wrist_roll_link"]
            # fk_request.robot_state.joint_state.position = grasp.pre_grasp_posture.points[0].positions
            # fk_request.robot_state.joint_state.name = grasp.pre_grasp_posture.joint_names
            # fk_response = self._fk_client(fk_request)

            grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                grasp.grasp_pose)


            # Check if grasp is from behind or too high or low
            if grasp_pose.pose.position.x > object_pose.pose.position.x or \
                grasp_pose.pose.position.z > (self.shelf_height + 0.24) or \
                grasp_pose.pose.position.z < self.shelf_height:
                continue

            transform = TransformStamped()
            transform.header.frame_id = 'base_footprint'
            transform.header.stamp = rospy.Time.now()
            transform.transform.translation = grasp_pose.pose.position
            transform.transform.rotation = grasp_pose.pose.orientation
            transform.child_frame_id = 'grasp'
            self._set_static_tf.wait_for_service()
            self._set_static_tf(transform)
            rospy.sleep(0.5)


            pre_grasp_pose = PoseStamped()
            pre_grasp_pose.header.stamp = rospy.Time(0)
            pre_grasp_pose.header.frame_id = "grasp"
            pre_grasp_pose.pose.position.x = -0.05
            pre_grasp_pose.pose.orientation.w = 1

            base_footprint_pre_grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_pose)


            rospy.loginfo(" ")
            rospy.loginfo(" ")

            grasp_dict = {}
            grasp_dict["pre_grasp"] = pre_grasp_pose_base_footprint
            grasp_dict["grasp"] = grasp_pose
            if not rotate:
                grasping_pairs.append(grasp_dict)

            # Make rotated wrist versions of all poses. 

            # Put poses in wrist roll frame

            pre_grasp_pose.header.stamp = rospy.Time(0)
            grasp_pose.header.stamp = rospy.Time(0)

            #wrist_frame_pre_grasp = self._tf_listener.transformPose('grasp',
            #                                                   pre_grasp_pose)
            #wrist_frame_grasp = self._tf_listener.transformPose('grasp',
            #                                                    grasp_pose)
            #type(pose) = geometry_msgs.msg.Pose


            quaternion = (
                pre_grasp_pose.pose.orientation.x,
                pre_grasp_pose.pose.orientation.y,
                pre_grasp_pose.pose.orientation.z,
                pre_grasp_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            roll = roll - 1.57

            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            #type(pose) = geometry_msgs.msg.Pose
            pre_grasp_pose.pose.orientation.x = quaternion[0]
            pre_grasp_pose.pose.orientation.y = quaternion[1]
            pre_grasp_pose.pose.orientation.z = quaternion[2]
            pre_grasp_pose.pose.orientation.w = quaternion[3]

            base_footprint_pre_grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_pose)
    
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            #type(pose) = geometry_msgs.msg.Pose
            quaternion = (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
        
            roll = roll - 1.57
        
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            #type(pose) = geometry_msgs.msg.Pose
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pose)
            grasp_dict = {}
            grasp_dict["pre_grasp"] = base_footprint_pre_grasp_pose

            grasp_dict["grasp"] = grasp_pose
            if rotate:
                grasping_pairs.append(grasp_dict)

        return grasping_pairs

    def generate_grasps(self, object_pose, bin_id):

        grasping_pairs = [] # list of pre_grasp/grasp dicts 

        self.shelf_height = self._shelf_heights[bin_id]

        # Pre-grasp: pose arm in front of bin

        pre_grasp_pose_target = PoseStamped()
        pre_grasp_pose_target.header.frame_id = 'base_footprint';

        if bin_id > 'C':
            rospy.loginfo('Not in the top row')
            pre_grasp_pose_target.pose.orientation.w = 1
            pre_grasp_pose_target.pose.position.x = self.pre_grasp_x_distance 
            pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y

            # go for centroid if it's vertically inside shelf
            if ((object_pose.pose.position.z > (self.shelf_height + self.half_gripper_height))
                and (object_pose.pose.position.z < (self.shelf_height + 0.15))):
                pre_grasp_pose_target.pose.position.z = object_pose.pose.position.z
            # otherwise, centroid is probably wrong, just use lowest possible grasp
            else:
                pre_grasp_pose_target.pose.position.z = self.shelf_height + \
                    self.half_gripper_height + self.pre_grasp_height

        else:
            rospy.loginfo('In top row')
            pre_grasp_pose_target.pose.orientation.x = 0.984
            pre_grasp_pose_target.pose.orientation.y = -0.013
            pre_grasp_pose_target.pose.orientation.z = 0.178
            pre_grasp_pose_target.pose.orientation.w = 0.028
            pre_grasp_pose_target.pose.position.x = 0.243 
            pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y
            pre_grasp_pose_target.pose.position.z = 1.508


        # Move gripper into bin
        grasp_pose_target = PoseStamped()
        grasp_pose_target.header.frame_id = 'base_footprint';
        if bin_id > 'C':

            rospy.loginfo('Not grasping from top row')
            grasp_pose_target.pose.orientation.w = 1
            grasp_pose_target.pose.position.x = \
                object_pose.pose.position.x - self.dist_to_palm
            grasp_pose_target.pose.position.y = object_pose.pose.position.y
            if ((object_pose.pose.position.z > (self.shelf_height + self.half_gripper_height))
                and (object_pose.pose.position.z < (self.shelf_height + 0.15))):
                grasp_pose_target.pose.position.z = object_pose.pose.position.z
            else:
                grasp_pose_target.pose.position.z = self.shelf_height + self.half_gripper_height


        else:
            rospy.loginfo('Grasping from top row')
            grasp_pose_target.pose.orientation.x = 0.996
            grasp_pose_target.pose.orientation.y = -0.016
            grasp_pose_target.pose.orientation.z = 0.080
            grasp_pose_target.pose.orientation.w = 0.027
            grasp_pose_target.pose.position.x = 0.431
            grasp_pose_target.pose.position.y = object_pose.pose.position.y
            grasp_pose_target.pose.position.z = 1.570


        grasp_dict = {}
        grasp_dict["pre_grasp"] = pre_grasp_pose_target
        grasp_dict["grasp"] = grasp_pose_target
        grasping_pairs.append(grasp_dict)

        grasp_action_client = actionlib.SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
        grasp_action_client.wait_for_server()
        goal = GenerateGraspsGoal()
        pose = object_pose.pose
        #type(pose) = geometry_msgs.msg.Pose
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
        goal.pose.position = object_pose.pose.position
        goal.pose.orientation = pose.orientation

        goal.width = 0.0

        options_1 = GraspGeneratorOptions()
        options_1.grasp_axis = options_1.GRASP_AXIS_Z
        options_1.grasp_direction = options_1.GRASP_DIRECTION_UP
        options_1.grasp_rotation = options_1.GRASP_ROTATION_FULL

        options_2 = GraspGeneratorOptions()
        options_2.grasp_axis = options_2.GRASP_AXIS_Z
        options_2.grasp_direction = options_2.GRASP_DIRECTION_DOWN
        options_2.grasp_rotation = options_2.GRASP_ROTATION_FULL

        goal.options.append(options_1)
        goal.options.append(options_2)
        # grasp_action_client.send_goal(goal)
        # grasp_action_client.wait_for_result()

        # grasps_result = grasp_action_client.get_result()

        # moveit_simple_grasps  = self.grasp_msg_to_poses(grasps_result.grasps)
        
        options_1 = GraspGeneratorOptions()
        options_1.grasp_axis = options_1.GRASP_AXIS_X
        options_1.grasp_direction = options_1.GRASP_DIRECTION_UP
        options_1.grasp_rotation = options_1.GRASP_ROTATION_FULL

        options_2 = GraspGeneratorOptions()
        options_2.grasp_axis = options_2.GRASP_AXIS_X
        options_2.grasp_direction = options_2.GRASP_DIRECTION_DOWN
        options_2.grasp_rotation = options_2.GRASP_ROTATION_FULL

        goal.options = []
        goal.options.append(options_1)
        goal.options.append(options_2)
        grasp_action_client.send_goal(goal)
        grasp_action_client.wait_for_result()

        grasps_result = grasp_action_client.get_result()

        moveit_simple_grasps  =  moveit_simple_grasps + self.grasp_msg_to_poses(
                                                                                grasps_result.grasps,
                                                                                object_pose, True)

        options_1 = GraspGeneratorOptions()
        options_1.grasp_axis = options_1.GRASP_AXIS_Y
        options_1.grasp_direction = options_1.GRASP_DIRECTION_UP
        options_1.grasp_rotation = options_1.GRASP_ROTATION_FULL

        options_2 = GraspGeneratorOptions()
        options_2.grasp_axis = options_2.GRASP_AXIS_Y
        options_2.grasp_direction = options_2.GRASP_DIRECTION_DOWN
        options_2.grasp_rotation = options_2.GRASP_ROTATION_FULL

        goal.options.append(options_1)
        goal.options.append(options_2)
        grasp_action_client.send_goal(goal)
        grasp_action_client.wait_for_result()

        grasps_result = grasp_action_client.get_result()

        mmoveit_simple_grasps  =  moveit_simple_grasps + self.grasp_msg_to_poses(
                                                                                grasps_result.grasps,
                                                                                object_pose, False)


        # Commented out for testing purposes
        grasping_pairs = grasping_pairs + moveit_simple_grasps

        return grasping_pairs

    def get_grasp_intersections(self, grasp):
        # just a placeholder, returns dict [pre_grasp, grasp, pre_grasp_reachable, grasp_reachable, grasp_quality]
        # may want to make a class
        viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
        viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')

        # Check if enough points will be in gripper
        rospy.loginfo("Just checking grasp quality") 
        y_offset = 0.005

        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        transform.transform.rotation = grasp["grasp"].pose.orientation
        transform.child_frame_id = 'grasp'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.5)

        points_in_box_request = BoxPointsRequest()
        points_in_box_request.frame_id = "grasp"
        points_in_box_request.cluster = self._cluster

        box_request = Box()
        box_request.min_x = self.dist_to_palm
        box_request.max_x = self.dist_to_fingertips
        box_request.min_y = -1 * self.gripper_palm_width/2 + y_offset
        box_request.max_y = self.gripper_palm_width/2 + y_offset
        box_request.min_z = -3 * self.gripper_finger_height/2
        box_request.max_z = 3 * self.gripper_finger_height/2
        points_in_box_request.boxes.append(box_request)

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'grasp'
        box_pose.pose.position.x = box_request.min_x + (box_request.max_x - box_request.min_x) / 2
        box_pose.pose.position.y = box_request.min_y + (box_request.max_y - box_request.min_y) / 2
        box_pose.pose.position.z = box_request.min_z + (box_request.max_z - box_request.min_z) / 2

        viz.publish_bounding_box(self._markers, box_pose, (box_request.max_x - box_request.min_x), 
            (box_request.max_y - box_request.min_y), (box_request.max_z - box_request.min_z),
            1.0, 0.0, 0.0, 0.5, 1)
        

        # Check for collisions with fingers
        # Left Finger
        l_finger_request = Box()
        l_finger_request.min_x = self.dist_to_palm
        l_finger_request.max_x = self.dist_to_fingertips
        l_finger_request.min_y = -1 * self.gripper_palm_width/2 - 0.025 + y_offset
        l_finger_request.max_y = -1 * self.gripper_palm_width/2 - 0.005 + y_offset
        l_finger_request.min_z = -1 * self.gripper_finger_height/2
        l_finger_request.max_z = self.gripper_finger_height/2
        points_in_box_request.boxes.append(l_finger_request)
        
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
        r_finger_request.min_x = self.dist_to_palm
        r_finger_request.max_x = self.dist_to_fingertips
        r_finger_request.min_y = self.gripper_palm_width/2 + 0.005 + y_offset
        r_finger_request.max_y = self.gripper_palm_width/2 + 0.025 + y_offset
        r_finger_request.min_z = -1 * self.gripper_finger_height/2
        r_finger_request.max_z = self.gripper_finger_height/2
        points_in_box_request.boxes.append(r_finger_request)
        
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
        points_in_box_request.boxes.append(palm_request)
        
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

        self._get_points_in_box.wait_for_service()
        box_response = self._get_points_in_box(points_in_box_request)
        rospy.loginfo("Number of points inside gripper: {}".format(box_response.num_points[0]))
        rospy.loginfo("Number of points inside left finger: {}"
                        .format(box_response.num_points[1]))
        rospy.loginfo("Number of points inside right finger: {}"
                        .format(box_response.num_points[2]))
        rospy.loginfo("Number of points inside palm: {}"
                        .format(box_response.num_points[3]))

        grasp["finger_collision_points"] = box_response.num_points[1] + box_response.num_points[2]
        grasp["palm_collision_points"] = box_response.num_points[3]
        grasp["points_in_gripper"] = box_response.num_points[0]

        if grasp["points_in_gripper"] >= self.min_points_in_gripper and \
            grasp["finger_collision_points"] <= self.max_finger_collision_points and \
            grasp["palm_collision_points"] <= self.max_palm_collision_points:
            grasp["grasp_quality"] = (grasp["points_in_gripper"] -  
                                      grasp["finger_collision_points"] - 
                                      grasp["palm_collision_points"])  
            rospy.loginfo("Evaluated good grasp")
        else:
            grasp["grasp_quality"] = 0.0
            rospy.loginfo("Evaluated bad grasp")
        rospy.loginfo("Grasp quality: " + str(grasp["grasp_quality"]))
        return grasp

    def check_reachable(self, grasp):

        # Check pre-grasp IK
        rospy.loginfo("Checking pre-grasp ik")
        #ik_request = GetPositionIKRequest()
        #ik_request.ik_request.group_name = "right_arm"
        #ik_request.ik_request.pose_stamped = grasp["pre_grasp"]
        #ik_response = self._ik_client(ik_request)

        #if ik_response.error_code.val == ik_response.error_code.SUCCESS:
        #    grasp["pre_grasp_reachable"] = True
        #else: 
        #    grasp["pre_grasp_reachable"] = False
        self._moveit_move_arm.wait_for_service()
        success_pre_grasp = self._moveit_move_arm(grasp["pre_grasp"],
                                                  0.01, 0.01, 0, 'right_arm', 
                                                  True).success
        grasp["pre_grasp_reachable"] = success_pre_grasp
        # Check grasp IK
        rospy.loginfo("Checking grasp ik")
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "right_arm"
        ik_request.ik_request.pose_stamped = grasp["grasp"]
        ik_response = self._ik_client(ik_request)

        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            grasp["grasp_reachable"] = True
        else:
            grasp["grasp_reachable"] = False

        return grasp

    def sort_grasps(self, grasps):
        sorted_grasps = sorted(grasps, key=lambda k: k['grasp_quality'])
        sorted_grasps.reverse()
        return sorted_grasps


    def get_reachable_grasp(self, grasp):

        # If pre-grasp not reachable, try
        if not grasp["pre_grasp_reachable"]:
            pre_grasp_offsets = [
                self.pre_grasp_attempt_separation * i
                for i in range(self.pre_grasp_attempts)
            ]
            rospy.loginfo("Pre-grasp not reachable")
            reachable = False
            for (idx, offset) in enumerate(pre_grasp_offsets):
                # transform pre-grasp into r_wrist_roll_link frame
                transformed_pose = self._tf_listener.transformPose('grasp',
                                                            grasp["pre_grasp"])
                # move it forward in x
                transformed_pose.pose.position.x = transformed_pose.pose.position.x - offset

                #check ik
                rospy.loginfo("Checking ik")
                #ik_request = GetPositionIKRequest()
                #ik_request.ik_request.group_name = "right_arm"
                #ik_request.ik_request.pose_stamped = transformed_pose
                #ik_response = self._ik_client(ik_request)

                #if reachable, set True, set new pre-grasp, break
                #if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                #    reachable = True
                pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        transformed_pose)
                grasp["pre_grasp"] = pose_in_base_footprint
                #    grasp["pre_grasp_reachable"] = True
                #    break 

                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(transformed_pose,
                                                  0.01, 0.01, 0, 'right_arm',
                                                  True).success
                grasp["pre_grasp_reachable"] = success_pre_grasp
                grasp["pre_grasp"] =  pose_in_base_footprint
                if success_pre_grasp:
                    rospy.loginfo("Found reachable.")
                    break
                    

        if not grasp["grasp_reachable"]:
            grasp_attempt_delta = \
                (self.dist_to_fingertips - self.dist_to_palm) / self.grasp_attempts
            grasp_attempt_offsets = [
                grasp_attempt_delta * i
                for i in range(self.grasp_attempts)
            ]
            rospy.loginfo("Grasp not reachable.")
            for (idx, offset) in enumerate(grasp_attempt_offsets):
                # transform grasp into r_wrist_roll_link frame
                transformed_pose = self._tf_listener.transformPose('grasp',
                                                            grasp["grasp"])

                # move it forward in x
                transformed_pose.pose.position.x = transformed_pose.pose.position.x - offset

                #check ik
                ik_request = GetPositionIKRequest()
                ik_request.ik_request.group_name = "right_arm"
                ik_request.ik_request.pose_stamped = transformed_pose
                ik_response = self._ik_client(ik_request)

                #if reachable, set True, set new grasp, evaluate grasp, append, break
                if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                    pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                            transformed_pose)
                    grasp["grasp"] = pose_in_base_footprint
                    grasp["grasp_reachable"] = True

                    # Check if shifted grasp still has object in gripper
                    grasp = self.get_grasp_intersections(grasp)
                    rospy.loginfo("Found reachable")
                    break

        return grasp

    def filter_grasps(self, grasps):
        # High quality grasps where the gripper is in the right position
        good_grasps = []
        # Good grasps that are reachable
        reachable_good_grasps = []
        num_reachable = 0
        num_good = 0
        attempts = 0
        
        for grasp in grasps:

            grasp = self.get_grasp_intersections(grasp)

            if grasp["points_in_gripper"] >= self.min_points_in_gripper and \
                grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                grasp["palm_collision_points"] > self.max_palm_collision_points:

                grasp_attempt_delta = 0.01
                grasp_attempts = 20
                grasp_attempt_offsets = [grasp_attempt_delta * i 
                                        for i in range(grasp_attempts)]
                transformed_pose = self._tf_listener.transformPose(
                                                                'grasp',
                                                                grasp["grasp"])
                # Good grasp but too many points in palm
                rospy.loginfo("Good grasp but too many points in the palm.")
                for (idx, offset) in enumerate(grasp_attempt_offsets):

                    rospy.loginfo("Backing off attempt {}.".format(idx))
                    # move it forward in x
                    transformed_pose.pose.position.x = \
                        transformed_pose.pose.position.x - offset

                    pose_in_base_footprint = self._tf_listener.transformPose(
                                                                'base_footprint',
                                                                transformed_pose)

                    grasp["grasp"] = pose_in_base_footprint

                    grasp = self.get_grasp_intersections(grasp)

                    if grasp["points_in_gripper"] >= self.min_points_in_gripper and \
                        grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                        grasp["palm_collision_points"] <= self.max_palm_collision_points:
                        rospy.loginfo("Good grasp.")
                        good_grasps.append(grasp)
                        break
                    if grasp["points_in_gripper"] == 0:
                        rospy.loginfo("Giving up on this grasp, no more points in gripper.")
                        break


            elif grasp["points_in_gripper"] >= self.min_points_in_gripper and \
                grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                grasp["palm_collision_points"] <= self.max_palm_collision_points:

                rospy.loginfo("Good grasp.")
                good_grasps.append(grasp)


        for grasp in good_grasps:
            grasp = self.check_reachable(grasp)
            if grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                rospy.loginfo("Grasp reachable")
                reachable_good_grasps.append(grasp)
            else:
                # Move and try to check IK
                rospy.loginfo("Trying to make grasp reachable.")
                grasp = self.get_reachable_grasp(grasp)
                if grasp["grasp_quality"] > self.min_grasp_quality and \
                    grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                    reachable_good_grasps.append(grasp)


        return self.sort_grasps(reachable_good_grasps)

    def execute_grasp(self, grasps):
        success_pre_grasp = False
        success_grasp = False
        for grasp in grasps:
            self._moveit_move_arm.wait_for_service()
            success_pre_grasp = self._moveit_move_arm(grasp["pre_grasp"], 
                                                    0.01, 0.01, 0, 'right_arm',
                                                    False).success
 
            if not success_pre_grasp:
                continue
            else:
                rospy.loginfo('Pre-grasp succeeeded')
                rospy.loginfo('Open Hand')
                self._set_grippers.wait_for_service()
 
                grippers_open = self._set_grippers(False, True, -1)

            self._move_arm_ik.wait_for_service()

            success_grasp = self._move_arm_ik(grasp["grasp"], MoveArmIkRequest().RIGHT_ARM).success

            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            success_grasp = True
            if success_grasp:
                rospy.loginfo('Grasp succeeded')
                rospy.loginfo('Close Hand')
                self._lookup_item.wait_for_service()
                lookup_response = self._lookup_item(item=userdata.current_target)
                item_model = lookup_response.model
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(open_left=False, open_right=False,
                                                   effort=item_model.grasp_effort)
                break
        if success_grasp:
            return True
        else:
            return False


    def execute(self, userdata):
        self._tts.publish('Grasping item')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

        self._cluster = userdata.clusters[0]

        # TODO(sksellio): check whether this works.
        #self._tf_listener.waitForTransform(
        #        'base_footprint',
        #        'bin_{}'.format(userdata.bin_id),
        #        rospy.Time(0),
        #        self._wait_for_transform_duration,
        #)

        # Get the pose of the target item in the base frame
        response = self._find_centroid(userdata.target_cluster)
        item_point = response.centroid
        item_pose = PoseStamped(
            header=Header(
                frame_id=item_point.header.frame_id,
                stamp=rospy.Time(0),
            ),
            pose=Pose(
                position=item_point.point,
                orientation=Quaternion(w=1, x=0, y=0, z=0),
            )
        )
        base_frame_item_pose = self._tf_listener.transformPose('base_footprint',
                                                                item_pose)

        rospy.loginfo(
            'Grasping item in bin {} from pose {}'
            .format(userdata.bin_id, base_frame_item_pose)
        )
        if userdata.debug:
            raw_input('(Debug) Press enter to continue >')

        # scene = moveit_commander.PlanningSceneInterface()
        # scene.remove_world_object('shelf')
        # self.add_shelf_mesh_to_scene(scene)

        grasping_pairs = self.generate_grasps(base_frame_item_pose, userdata.bin_id)
        filtered_grasps = self.filter_grasps(grasping_pairs)
        if len(filtered_grasps) > 0:
            success_grasp = self.execute_grasp(filtered_grasps)
        else:
            rospy.loginfo('No good grasps found')
            self._tts.publish('No grasps found.')
            return outcomes.GRASP_FAILURE


        if not success_grasp:
            rospy.loginfo('Grasping failed')
            self._tts.publish('Grasping failed. Giving up.')
            return outcomes.GRASP_FAILURE
        else:
            rospy.loginfo('Grasping succeeded')
            self._tts.publish('Grasping succeeded.')
            return outcomes.GRASP_SUCCESS
