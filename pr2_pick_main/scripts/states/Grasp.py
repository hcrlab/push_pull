import actionlib
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Point, PointStamped
import json
import math
import moveit_commander
from moveit_msgs.msg import Grasp, PlanningScene, PlanningSceneComponents 
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetPlanningScene
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal
from moveit_simple_grasps.msg import GraspGeneratorOptions
import os
from pr2_pick_main import handle_service_exceptions
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
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest, PlanarPrincipalComponentsRequest, \
    DeleteStaticTransformRequest


class Grasp(smach.State):
    ''' Grasps an item in the bin. '''
    name = 'GRASP'
    bounding_box_marker_id = 46976
    bounding_box_pose_marker_id = 46977
    # this and this + 1 are the corner ids
    corner_marker_id = 46978


    # How many pre-grasp gripper positions to attempt
    pre_grasp_attempts = 3
    # Separation in meters between attempted pre-grasp positions
    pre_grasp_attempt_separation = 0.01
    # how many grasp gripper positions to attempt
    grasp_attempts = 20

    # desired distance from palm frame to object centroid
    pre_grasp_x_distance = 0.37

    pre_grasp_offset = 0.08

    # approximately half hand thickness
    half_gripper_height = 0.03
    # approximate distance from palm frame origin to palm surface
    dist_to_palm = 0.11
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
    min_points_in_gripper = 100

    # max number of points in cluster that can intersect with fingers
    max_finger_collision_points = 10

    max_palm_collision_points = 10

    shelf_bottom_height = None
    shelf_height = None
    shelf_wdith = None

    low_object  = False

    top_shelf = False

    grasp_num = 0

    ik_timeout = rospy.Duration(3.0)

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id', 'debug', 'target_cluster', 'current_target',
                        'item_model', 'target_descriptor']
        )

        self._find_centroid = services['find_centroid']
        self._set_grippers = services['set_grippers']
        self._tuck_arms = services['tuck_arms']
        self._moveit_move_arm = services['moveit_move_arm']
        self._tts = services['tts']
        self._tf_listener = services['tf_listener']
        self._im_server = services['interactive_marker_server']
        self._set_static_tf = services['set_static_tf']
        self._delete_static_tf = services['delete_static_tf']
        self._markers = services['markers']
        self._move_arm_ik = services['move_arm_ik']

        self._fk_client = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self._ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._get_points_in_box = rospy.ServiceProxy('perception/get_points_in_box', BoxPoints)
        self._lookup_item = services['lookup_item']
        self._get_planar_pca = services['get_planar_pca']

        self._wait_for_transform_duration = rospy.Duration(5.0)

        self._cluster = None
        self._debug = False

        # Shelf heights

        self._shelf_bottom_height_a_c = 1.57
        self._shelf_bottom_height_d_f = 1.35
        self._shelf_bottom_height_g_i = 1.12
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
            'B': 0.30,
            'C': 0.25,
            'D': 0.25, 
            'E': 0.30,
            'F': 0.25,
            'G': 0.25, 
            'H': 0.30, 
            'I': 0.25, 
            'J': 0.25, 
            'K': 0.30, 
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

    def get_box_ends(self, item_descriptor):
        ''' Get the ends of the bounding box from the given descriptor '''
        bounding_box = item_descriptor.planar_bounding_box

        # First, figure out whether the y-axis points roughly along or opposite
        # the cluster's y-axis

        # get euler angles and normalize
        orientation = bounding_box.pose.pose.orientation
        (yaw, pitch, roll) = tf.transformations.euler_from_quaternion(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        )
        yaw = -yaw
        # if fabs(roll) + 1e-4 > math.pi and fabs(roll) - 1e-4 < math.pi:
        #     roll -= math.pi
        #     pitch = -pitch
        #     yaw = -yaw
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw <= -math.pi:
            yaw += 2 * math.pi

        rospy.loginfo('Sanity check: yaw should be the only nonzero')
        rospy.loginfo('roll {}, pitch {}, yaw {}'.format(roll, pitch, yaw))
        rospy.loginfo('Bounding box dimensions {} {} {}'
                      .format(bounding_box.dimensions.x,
                              bounding_box.dimensions.y,
                              bounding_box.dimensions.z)
                      )

        # get yaw sign
        # make sure it's in the range (-pi, pi]
        y_sign = 1.0
        if yaw < 0:
            y_sign = -1.0

        # relative to the bounding box's pose
        ends_in_item_frame = [
            Point(
                x=x_sign * bounding_box.dimensions.x / 2.0,
                y=y_sign * bounding_box.dimensions.y / 2.0,
                z=bounding_box.dimensions.z / 2.0,
            )
            for x_sign in [1.0, -1.0]
        ]

        # transform to the cluster's pose
        position = bounding_box.pose.pose.position
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        ends = [
            Point(
                x=point.x * cos_yaw - point.y * sin_yaw + position.x,
                y=point.x * sin_yaw + point.y * cos_yaw + position.y,
                z=point.z + position.z,
            )
            for point in ends_in_item_frame
        ]

        # visualize bounding box, pose, and ends
        viz.publish_bounding_box(
            self._markers,
            bounding_box.pose,
            bounding_box.dimensions.x,
            bounding_box.dimensions.y,
            bounding_box.dimensions.z,
            0.7, 0, 0, 0.25,
            self.bounding_box_marker_id,
        )
        viz.publish_pose(
            self._markers,
            bounding_box.pose,
            0, 0.7, 0, 0.5,
            self.bounding_box_pose_marker_id,
        )

        """
        for idx, end in enumerate(ends):
            viz.publish_point(
                self._markers,
                bounding_box.pose.header.frame_id,
                end,
                0.0, 0.0, 0.7, 0.5,
                self.corner_marker_id + idx,
            )
        """

        point_stamped_ends = []
        for end in ends:
            new_point = PointStamped()
            new_point.header.frame_id = bounding_box.pose.header.frame_id
            new_point.point = end
            point_stamped_ends.append(new_point)

        return point_stamped_ends


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
            rospy.sleep(0.5)


            pre_grasp_pose = PoseStamped()
            pre_grasp_pose.header.stamp = rospy.Time(0)
            pre_grasp_pose.header.frame_id = "grasp"
            pre_grasp_pose.pose.position.x = -1 * self.pre_grasp_offset
            pre_grasp_pose.pose.orientation.w = 1

            base_footprint_pre_grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_pose)

            rospy.loginfo(" ")
            rospy.loginfo(" ")

            grasp_dict = {}
            grasp_dict["pre_grasp"] = base_footprint_pre_grasp_pose
            grasp_dict["grasp"] = grasp_pose
            if not rotate:
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
            if rotate:
                grasping_pairs.append(grasp_dict)

        return grasping_pairs


    def modify_grasp(self, pose, r, p, y, x, y_pos, z):
        """
        Apply rotation and translation to pose and return
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

        pose_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                pose)

        # pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
        #                                                         pose)

        # Check if within bin_width
        if pose_in_bin_frame.pose.position.y > ((self.shelf_width/2) - (self.gripper_palm_width + 0.04)/2):
            pose_in_bin_frame.pose.position.y = (self.shelf_width/2) - (self.gripper_palm_width + 0.04)/2
        elif pose_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
            pose_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
        pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        pose_in_bin_frame)

        if rotated:
            gripper_offset = self.gripper_palm_width/2
        else:
            gripper_offset = self.half_gripper_height

        if ((pose_in_base_footprint.pose.position.z > (self.shelf_bottom_height + gripper_offset))
            and (pose_in_base_footprint.pose.position.z < (self.shelf_bottom_height + self.shelf_height - gripper_offset))):
            pose_in_base_footprint.pose.position.z = pose_in_base_footprint.pose.position.z
        else:
            pose_in_base_footprint.pose.position.z = self.shelf_bottom_height + gripper_offset + 0.02

        pose_in_frame = self._tf_listener.transformPose(frame,
                                                        pose_in_bin_frame)

        return pose_in_frame, pose_in_base_footprint

    def check_pose_within_bounds(self, pose, shelf_bottom_height, shelf_height, 
                                    shelf_width, bin_id, frame, rotated):

        in_bounds = True
        pose_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                pose)

        # pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
        #                                                         pose)

        # Check if within bin_width
        if pose_in_bin_frame.pose.position.y > ((self.shelf_width/2) - (self.gripper_palm_width + 0.0)/2) and pose_in_bin_frame.pose.position.x > 0:
            in_bounds = False
            rospy.loginfo("Pose >  y bounds")
            rospy.loginfo("Y pose: {}".format(pose_in_bin_frame.pose.position.y))
            rospy.loginfo("Bound: {}".format(((self.shelf_width/2) - (self.gripper_palm_width + 0.0)/2)))

        elif pose_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.0)/2) and pose_in_bin_frame.pose.position.x > 0:
            in_bounds = False
            rospy.loginfo("Pose <  y bounds")
            rospy.loginfo("Y pose: {}".format(pose_in_bin_frame.pose.position.y))
            rospy.loginfo("Bound: {}".format(-1*self.shelf_width/2 + (self.gripper_palm_width + 0.01)/2))
        pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                        pose_in_bin_frame)

        if rotated:
            gripper_offset = self.gripper_palm_width/2
        else:
            gripper_offset = self.half_gripper_height

        if ((pose_in_base_footprint.pose.position.z > (self.shelf_bottom_height + gripper_offset))
            and (pose_in_base_footprint.pose.position.z < (self.shelf_bottom_height + self.shelf_height - gripper_offset))):
            pose_in_base_footprint.pose.position.z = pose_in_base_footprint.pose.position.z
        else:
            #if pose_in_bin_frame.pose.position.x > 0:
            in_bounds = False
            rospy.loginfo("Pose not within z bounds")
            rospy.loginfo("Z pose: {}".format(pose_in_base_footprint.pose.position.z))
            rospy.loginfo("Bound: {}".format((self.shelf_bottom_height + gripper_offset)))
            rospy.loginfo("Bound: {}".format((self.shelf_bottom_height + self.shelf_height - gripper_offset)))

        pose_in_frame = self._tf_listener.transformPose(frame,
                                                        pose_in_bin_frame)

        return in_bounds



    def get_pca_aligned_grasps(self, bounding_box, axes_poses, bin_id):
        """
        Generate grasp and pre-grasp poses aligned with axes from PCA
        """

        # get target edge of bounding box 
        ends = self.get_box_ends(self.target_descriptor)
        
        # get closest end from ends
        new_ends = []
        for end in ends:
            new_end = self._tf_listener.transformPoint('base_footprint',
                                                                end)
            new_ends.append(new_end)

        ends = new_ends

        rospy.loginfo("Closest ends of bounding box: {}".format(ends))
        closest_end = ends[0]
        y_offset = -0.045
        if ends[0].point.x > ends[1].point.x:
            closest_end = ends[1]
            if ends[0].point.y < ends[1].point.y:
                y_offset = 0.035
        else:
            if ends[0].point.y > ends[1].point.y:
                y_offset = 0.035

        object_pose = bounding_box.pose

        object_pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                object_pose)

        grasps = []
        # make grasp centred on closest end
        grasp_end = PoseStamped()
        grasp_end.header.stamp = rospy.Time(0)
        grasp_end.header.frame_id = "base_footprint"
        grasp_end.pose.orientation.w = 1
        grasp_end.pose.position.x = closest_end.point.x - self.dist_to_palm
        grasp_end.pose.position.y = closest_end.point.y
        grasp_end.pose.position.z = object_pose_in_base_footprint.pose.position.z

        pre_grasp_end = PoseStamped()
        pre_grasp_end.header.stamp = rospy.Time(0)
        pre_grasp_end.header.frame_id = "base_footprint"
        pre_grasp_end.pose.orientation.w = 1
        pre_grasp_end.pose.position.x = self.pre_grasp_x_distance
        pre_grasp_end.pose.position.y = closest_end.point.y
        pre_grasp_end.pose.position.z = object_pose_in_base_footprint.pose.position.z

        grasp_in_bounds = self.check_pose_within_bounds(grasp_end,
                                                                self.shelf_bottom_height, self.shelf_height,
                                                                self.shelf_width, bin_id, 'base_footprint', False)
        if grasp_in_bounds:
            grasp_dict = {}
            grasp_dict['grasp'] = grasp_end
            grasp_dict['pre_grasp'] = pre_grasp_end
            grasp_dict["id"] = self.grasp_num
            self.grasp_num += 1

            grasps.append(grasp_dict)
            rospy.loginfo("Current info: {}".format(grasps))

        
        rospy.loginfo("Chosen end: {}".format(closest_end))
        rospy.loginfo("Offset is: {}".format(y_offset))
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
            rospy.sleep(0.5)
            self._tf_listener.waitForTransform("object_axis", "bin_" + str(bin_id), rospy.Time(0), rospy.Duration(4.0))

            closest_end = self._tf_listener.transformPoint('object_axis',
                                                                closest_end)


            # Transform bounding box info into frame of PCA axes
            bbox_pose_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                object_pose)
            object_pose = self._tf_listener.transformPose('base_footprint',
                                                                object_pose)
            grasp_in_axis_frame = PoseStamped()
            grasp_in_axis_frame.header.stamp = rospy.Time(0)
            grasp_in_axis_frame.header.frame_id = 'object_axis'
            grasp_in_axis_frame.pose.orientation.w = 1
            grasp_in_axis_frame.pose.position.x = -1 * self.dist_to_palm
            grasp_in_axis_frame.pose.position.y = closest_end.point.y
            grasp_in_axis_frame.pose.position.z = bbox_pose_axis_frame.pose.position.z

            # Transform into base_footprint
            grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                grasp_in_axis_frame)

            if grasp_in_base_footprint.pose.position.x > object_pose.pose.position.x:
                # Wrong way along axis
                grasp_in_axis_frame.pose.position.y -= y_offset
                grasp_in_axis_frame.pose.position.x = self.dist_to_palm
                grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                grasp_in_axis_frame)
                
                # Rotate grasp and pre_grasp 180 degrees
                grasp_in_base_footprint = self.modify_grasp(grasp_in_base_footprint, 
                                                                0, 0, -3.14, 0, 0, 0)
                grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                grasp_in_base_footprint)
                """
                pre_grasp_in_base_footprint = PoseStamped()
                pre_grasp_in_base_footprint.header.stamp = rospy.Time(0)
                pre_grasp_in_base_footprint.header.frame_id = "base_footprint"
                pre_grasp_in_base_footprint.pose.position.x = self.pre_grasp_x_distance
                pre_grasp_in_base_footprint.pose.position.y = grasp_in_base_footprint.pose.position.y
                pre_grasp_in_base_footprint.pose.position.z = grasp_in_base_footprint.pose.position.z
                """
                # grasp_in_base_footprint.pose.orientation = \
                #        pre_grasp_in_base_footprint.pose.orientation

                pre_grasp_in_axis_frame = PoseStamped()
                pre_grasp_in_axis_frame.header.frame_id = 'object_axis'
                pre_grasp_in_axis_frame.pose.position.y = grasp_in_axis_frame.pose.position.y
                pre_grasp_in_axis_frame.pose.position.x = grasp_in_axis_frame.pose.position.x + self.pre_grasp_offset
                pre_grasp_in_axis_frame.pose.position.z = grasp_in_axis_frame.pose.position.z
                pre_grasp_in_axis_frame.pose.orientation = grasp_in_axis_frame.pose.orientation
                pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_in_axis_frame)

                grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                grasp_in_base_footprint)
                #grasp_in_bin_frame.pose.position.y -= 0.015

                grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                grasp_in_bin_frame)

                # # Check if within bin_width
                # if pre_grasp_in_axis_frame.pose.position.y > ((self.shelf_width/2) - (self.gripper_palm_width + 0.04)/2):
                #     pre_grasp_in_axis_frame.pose.position.y = (self.shelf_width/2) - (self.gripper_palm_width + 0.04)/2
                # elif pre_grasp_in_axis_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
                #     pre_grasp_in_axis_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
                # pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                #                                                 pre_grasp_in_axis_frame)

                # if ((object_pose.pose.position.z > (self.shelf_bottom_height + self.half_gripper_height))
                #     and (object_pose.pose.position.z < (self.shelf_bottom_height + self.shelf_height - self.half_gripper_height))):
                #     grasp_pose_target.pose.position.z = object_pose.pose.position.z
                # else:
                #     grasp_pose_target.pose.position.z = self.shelf_bottom_height + self.half_gripper_height

                # pre_grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                #                                                 pre_grasp_in_axis_frame)

                """
                pre_grasp_in_axis_frame, pre_grasp_in_base_footprint = self.move_pose_within_bounds(pre_grasp_in_axis_frame, 
                                                                self.shelf_bottom_height, self.shelf_height, 
                                                                self.shelf_width, bin_id, 'object_axis', False)
                """

                grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                grasp_in_base_footprint)



            else:
                grasp_in_axis_frame.pose.position.y += y_offset
                grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                grasp_in_axis_frame)                
                """
                pre_grasp_in_base_footprint = PoseStamped()
                pre_grasp_in_base_footprint.header.stamp = rospy.Time(0)
                pre_grasp_in_base_footprint.header.frame_id = "base_footprint"
                pre_grasp_in_base_footprint.pose.position.x = self.pre_grasp_x_distance
                pre_grasp_in_base_footprint.pose.position.y = grasp_in_base_footprint.pose.position.y
                pre_grasp_in_base_footprint.pose.position.z = grasp_in_base_footprint.pose.position.z
                
                pre_grasp_in_axis_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                pre_grasp_in_base_footprint)
                """

                pre_grasp_in_axis_frame = PoseStamped()
                pre_grasp_in_axis_frame.header.frame_id = 'object_axis'
                pre_grasp_in_axis_frame.pose.position.y = grasp_in_axis_frame.pose.position.y
                pre_grasp_in_axis_frame.pose.position.x = grasp_in_axis_frame.pose.position.x - self.pre_grasp_offset
                pre_grasp_in_axis_frame.pose.position.z = grasp_in_axis_frame.pose.position.z
                pre_grasp_in_axis_frame.pose.orientation = grasp_in_axis_frame.pose.orientation
                pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                pre_grasp_in_axis_frame)

                # # Check if within bin_width
                # if pre_grasp_in_axis_frame.pose.position.y > ((self.shelf_width/2) - (self.gripper_palm_width + 0.04)/2):
                #     pre_grasp_in_axis_frame.pose.position.y = (self.shelf_width/2) - (self.gripper_palm_width + 0.04)/2
                # elif pre_grasp_in_axis_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
                #     pre_grasp_in_axis_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
                # pre_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                #                                                 pre_grasp_in_axis_frame)
                # pre_grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                #                                                 pre_grasp_in_axis_frame)

                """
                pre_grasp_in_axis_frame, pre_grasp_in_base_footprint = self.move_pose_within_bounds(pre_grasp_in_axis_frame, 
                                                                self.shelf_bottom_height, self.shelf_height, 
                                                                self.shelf_width, bin_id, 'object_axis', False)
                """
                grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                grasp_in_base_footprint)
                
                #grasp_in_bin_frame.pose.position.y -= 0.015

                grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                grasp_in_bin_frame)
                grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                grasp_in_base_footprint)

                
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

            # Make rotated version
            rolled_pre_grasp_in_axis_frame = self.modify_grasp(pre_grasp_in_axis_frame,
                                                                 -1.57, 0, 0, 0, 0, 0)

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
                rospy.loginfo("Rolled grasp not in bounds")
                rolled_grasp_in_base_footprint = self.modify_grasp(rolled_grasp_in_base_footprint,
                                                                        0, 0, 0, 0, 0, 0.07)
                rolled_grasp_in_axis_frame = self._tf_listener.transformPose('object_axis', 
                                                                        rolled_grasp_in_base_footprint)
                rolled_grasp_in_axis_frame = self.modify_grasp(rolled_grasp_in_axis_frame,
                                                                 0, 3.14/6, 0, 0, 0, 0)
                rolled_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                               rolled_grasp_in_axis_frame)

                grasp_in_bounds = self.check_pose_within_bounds(rolled_grasp_in_axis_frame,
                                                                self.shelf_bottom_height, self.shelf_height,
                                                                self.shelf_width, bin_id, 'object_axis', True)

            if grasp_in_bounds and ('rolled' in self.allowed_grasps):

                rospy.loginfo("Adding rolled grasp")

                grasp_dict = {}
                grasp_dict["grasp"] = rolled_grasp_in_base_footprint
                grasp_dict["pre_grasp"] = rolled_pre_grasp_in_base_footprint
                grasp_dict["rolled"] = True
                grasp_dict["id"] = self.grasp_num
                self.grasp_num += 1

                grasps.append(grasp_dict)

            # Make pitched down
            pitched_grasp_in_axis_frame = self.modify_grasp(grasp_in_axis_frame,
                                                             0, 3.14/6, 0, 0, 0, 0.0)
            pitched_grasp_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                pitched_grasp_in_axis_frame)

            pitched_grasp_in_base_footprint = self.modify_grasp(pitched_grasp_in_base_footprint,
                                                             0, 0, 0, 0, 0, 0.07)

            pitched_grasp_in_axis_frame = self._tf_listener.transformPose('object_axis',
                                                                pitched_grasp_in_base_footprint)

            grasp_in_bounds = self.check_pose_within_bounds(pitched_grasp_in_axis_frame, 
                                                                self.shelf_bottom_height, self.shelf_height, 
                                                                self.shelf_width, bin_id, 'object_axis', False)
            if grasp_in_bounds and ('rolled' in self.allowed_grasps):
                grasp_dict = {}
                grasp_dict["grasp"] = pitched_grasp_in_base_footprint
                grasp_dict["pre_grasp"] = pre_grasp_in_base_footprint
                grasp_dict["pitched_down"] = True
                grasp_dict["id"] = self.grasp_num
                self.grasp_num += 1
             
                grasps.append(grasp_dict)

            self._delete_static_tf.wait_for_service()
            req = DeleteStaticTransformRequest()
            req.parent_frame = "bin_" + str(bin_id)
            req.child_frame = "object_axis"
            self._delete_static_tf(req)

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

        if not self.top_shelf:
            rospy.loginfo('Not in the top row')
            pre_grasp_pose_target.pose.orientation.w = 1
            pre_grasp_pose_target.pose.position.x = self.pre_grasp_x_distance 
            pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y

            # # go for centroid if it's vertically inside shelf
            # if ((object_pose.pose.position.z > (self.shelf_bottom_height + self.half_gripper_height))
            #     and (object_pose.pose.position.z < (self.shelf_bottom_height + self.shelf_height - self.half_gripper_height))):
            #     pre_grasp_pose_target.pose.position.z = object_pose.pose.position.z
            # # otherwise, centroid is probably wrong, just use lowest possible grasp
            # else:
            #     pre_grasp_pose_target.pose.position.z = self.shelf_bottom_height + \
            #         self.half_gripper_height + self.pre_grasp_height
            #     self.low_object = True

            # pre_grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
            #                                                     pre_grasp_pose_target)
            # # Check if within bin_width
            # if pre_grasp_in_bin_frame.pose.position.y > (self.shelf_width/2  - (self.gripper_palm_width + 0.04)/2):
            #     pre_grasp_in_bin_frame.pose.position.y = (self.shelf_width/2 - (self.gripper_palm_width + 0.04)/2)
            # elif pre_grasp_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
            #     pre_grasp_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
            # pre_grasp_pose_target = self._tf_listener.transformPose('base_footprint',
            #                                                 pre_grasp_in_bin_frame)

            pre_grasp_in_bin_frame, pre_grasp_pose_target = self.move_pose_within_bounds(pre_grasp_pose_target, self.shelf_bottom_height, self.shelf_height, 
                                    self.shelf_width, bin_id, 'bin_' + str(bin_id), False)


        else:
            rospy.loginfo('In top row')
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
            if pre_grasp_in_bin_frame.pose.position.y > (self.shelf_width/2  - (self.gripper_palm_width + 0.04)/2):
                pre_grasp_in_bin_frame.pose.position.y = (self.shelf_width/2 - (self.gripper_palm_width + 0.04)/2)
            elif pre_grasp_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
                pre_grasp_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
            pre_grasp_pose_target = self._tf_listener.transformPose('base_footprint',
                                                            pre_grasp_in_bin_frame)



        # Move gripper into bin
        grasp_pose_target = PoseStamped()
        grasp_pose_target.header.frame_id = 'base_footprint';
        if not self.top_shelf:

            rospy.loginfo('Not grasping from top row')
            grasp_pose_target.pose.orientation.w = 1
            grasp_pose_target.pose.position.x = \
                object_pose.pose.position.x - self.dist_to_palm
            grasp_pose_target.pose.position.y = object_pose.pose.position.y
            # if ((object_pose.pose.position.z > (self.shelf_bottom_height + self.half_gripper_height))
            #     and (object_pose.pose.position.z < (self.shelf_bottom_height + self.shelf_height - self.half_gripper_height))):
            #     grasp_pose_target.pose.position.z = object_pose.pose.position.z
            # else:
            #     grasp_pose_target.pose.position.z = self.shelf_bottom_height + self.half_gripper_height
            #     self.low_object = True

            # grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
            #                                                     grasp_pose_target)
            # # Check if within bin_width
            # if grasp_in_bin_frame.pose.position.y > (self.shelf_width/2  - (self.gripper_palm_width + 0.04)/2):
            #     grasp_in_bin_frame.pose.position.y = (self.shelf_width/2 - (self.gripper_palm_width + 0.04)/2)
            # elif grasp_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
            #     grasp_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
            # grasp_pose_target = self._tf_listener.transformPose('base_footprint',
            #                                                 grasp_in_bin_frame)

            grasp_in_bin_frame, grasp_pose_target = self.move_pose_within_bounds(grasp_pose_target, self.shelf_bottom_height, self.shelf_height, 
                                    self.shelf_width, bin_id, 'bin_' + str(bin_id), False)

        else:
            rospy.loginfo('Grasping from top row')
            grasp_pose_target.pose.orientation.x = 0.996
            grasp_pose_target.pose.orientation.y = -0.016
            grasp_pose_target.pose.orientation.z = 0.080
            grasp_pose_target.pose.orientation.w = 0.027
            grasp_pose_target.pose.position.x = 0.431
            grasp_pose_target.pose.position.y = object_pose.pose.position.y
            grasp_pose_target.pose.position.z = 1.580 # used to be 1.57

            grasp_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                grasp_pose_target)
            # Check if within bin_width
            if grasp_in_bin_frame.pose.position.y > (self.shelf_width/2  - (self.gripper_palm_width + 0.04)/2):
                grasp_in_bin_frame.pose.position.y = (self.shelf_width/2 - (self.gripper_palm_width + 0.04)/2)
            elif grasp_in_bin_frame.pose.position.y < (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2):
                grasp_in_bin_frame.pose.position.y = (-1*self.shelf_width/2 + (self.gripper_palm_width + 0.04)/2)
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
        #grasp_action_client.send_goal(goal)
        #grasp_action_client.wait_for_result()

        #grasps_result = grasp_action_client.get_result()

        #moveit_simple_grasps  =  moveit_simple_grasps + self.grasp_msg_to_poses(
        #                                                                        grasps_result.grasps,
        #                                                                        object_pose, True)

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
        #grasp_action_client.send_goal(goal)
        #grasp_action_client.wait_for_result()

        #grasps_result = grasp_action_client.get_result()

        #moveit_simple_grasps  =  moveit_simple_grasps + self.grasp_msg_to_poses(
        #                                                                        grasps_result.grasps,
        #                                                                        object_pose, False)

        # Removing moveit generated grasps for now
        moveit_simple_grasps = []

        # Get pca aligned grasps
        rospy.loginfo("Getting pca grasps")
        object_pose_in_bin_frame = self._tf_listener.transformPose('bin_' + str(bin_id),
                                                                object_pose)

        self._get_planar_pca.wait_for_service()
        pca_resp = self._get_planar_pca(self._cluster)
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


        # Commented out for testing purposes
        grasping_pairs = grasping_pairs + moveit_simple_grasps + pca_grasps

        rospy.loginfo("Number of grasps: {}".format(len(grasping_pairs)))

        return grasping_pairs

    def get_grasp_intersections(self, grasp):
        """
        Check for intersections between gripper and point cloud
        """
        # may want to make a class
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
        box_request.min_z = -1 * self.gripper_finger_height/2
        box_request.max_z = 1 * self.gripper_finger_height/2
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
        l_finger_request.min_x = self.dist_to_palm - 0.02
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
        r_finger_request.min_x = self.dist_to_palm - 0.02
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
            rospy.loginfo("Grasp quality: " + str(grasp["grasp_quality"]))
            if 'front_grasp' in grasp:
                #rospy.loginfo("Front grasp! Making Quality Max = 10000")
                grasp["grasp_quality"] = grasp["grasp_quality"]  #self.max_grasp_quality
            if ('pitched_down' in grasp) and self.low_object:
                rospy.loginfo("Low object!")
                grasp["grasp_quality"] = 1.5 * grasp["grasp_quality"]  
        else:
            grasp["grasp_quality"] = 0.0
            rospy.loginfo("Evaluated bad grasp")
            rospy.loginfo("Grasp quality: " + str(grasp["grasp_quality"]))

        if self._debug:
            raw_input('(Debug) Press enter to continue >')

        return grasp

    def check_reachable(self, grasp):

        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        transform.transform.rotation = grasp["grasp"].pose.orientation
        transform.child_frame_id = 'grasp'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.5)

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
        rospy.loginfo("Pre-grasp reachable: {}".format(success_pre_grasp))
        grasp["pre_grasp_reachable"] = success_pre_grasp
        # Check grasp IK
        rospy.loginfo("Checking grasp ik")

        self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
        rospy.wait_for_service('/get_planning_scene', 10.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = get_planning_scene(request)

        acm = response.scene.allowed_collision_matrix
        if not 'bbox' in acm.default_entry_names:
            # add button to allowed collision matrix 
            acm.default_entry_names += ['bbox']
            acm.default_entry_values += [True]

            planning_scene_diff = PlanningScene(
                is_diff=True,
                allowed_collision_matrix=acm)

            #self._pubPlanningScene.publish(planning_scene_diff)
            #rospy.sleep(1.0)

        #success_grasp = self._moveit_move_arm(grasp["grasp"],
        #                                          0.01, 0.01, 0, 'right_arm', 
        #                                         True).success
        #rospy.loginfo("Grasp reachable: {}".format(success_grasp))
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

        rospy.loginfo("Grasp reachable: {}".format(grasp["grasp_reachable"]))
        return grasp

    def sort_grasps(self, grasps):
        sorted_grasps = sorted(grasps, key=lambda k: k['grasp_quality'])
        sorted_grasps.reverse()
        return sorted_grasps


    def get_reachable_grasp(self, grasp):


        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        transform.transform.rotation = grasp["grasp"].pose.orientation
        transform.child_frame_id = 'grasp'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(0.5)

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

                rotated = False
                if 'rolled' in grasp:
                    rotated = grasp['rolled']
                    
                pre_grasp_in_bounds = self.check_pose_within_bounds(transformed_pose, 
                                                            self.shelf_bottom_height, self.shelf_height, 
                                                            self.shelf_width, self.bin_id, 'base_footprint', rotated)
                if not pre_grasp_in_bounds:
                    break

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
                viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')
                #    grasp["pre_grasp_reachable"] = True
                #    break 

                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(transformed_pose,
                                                  0.01, 0.1, 0, 'right_arm',
                                                  True).success
                grasp["pre_grasp_reachable"] = success_pre_grasp
                grasp["pre_grasp"] =  pose_in_base_footprint
                if success_pre_grasp:
                    rospy.loginfo("Found reachable pre-grasp.")
                    rospy.loginfo("Pre_grasp: {}".format(grasp["pre_grasp"]))
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

                rotated = False
                if 'rolled' in grasp:
                    rotated = grasp['rolled']
                    
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
                self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
                rospy.wait_for_service('/get_planning_scene', 10.0)
                get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
                request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
                response = get_planning_scene(request)

                acm = response.scene.allowed_collision_matrix
                if not 'bbox' in acm.default_entry_names:
                    # add button to allowed collision matrix 
                    acm.default_entry_names += ['bbox']
                    acm.default_entry_values += [True]

                    planning_scene_diff = PlanningScene(
                        is_diff=True,
                        allowed_collision_matrix=acm)

                    #self._pubPlanningScene.publish(planning_scene_diff)
                    #rospy.sleep(1.0)

                #success_grasp = self._moveit_move_arm(grasp["grasp"],
                #                                          0.01, 0.01, 0, 'right_arm',
                #                                         True).success

                #if reachable, set True, set new grasp, evaluate grasp, append, break
                if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                    #if success_grasp:
                    pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                            transformed_pose)
                    grasp["grasp"] = pose_in_base_footprint
                    grasp["grasp_reachable"] = True

                    # Check if shifted grasp still has object in gripper
                    grasp = self.get_grasp_intersections(grasp)
                    rospy.loginfo("Found reachable grasp.")
                    rospy.loginfo("Grasp: {}".format(grasp["grasp_reachable"]))
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
        
        for (idx, grasp) in enumerate(grasps):
            rospy.loginfo("Now evaluating grasp: {}".format(idx))

            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')



            grasp = self.get_grasp_intersections(grasp)

            rospy.loginfo("Evaluating Grasp {}".format(grasp['id']))

            if grasp["finger_collision_points"] <= self.max_finger_collision_points and \
                grasp["palm_collision_points"] > self.max_palm_collision_points:

                grasp_attempt_delta = 0.01
                grasp_attempts = 20
                grasp_attempt_offsets = [grasp_attempt_delta * i 
                                        for i in range(grasp_attempts)]
                rospy.loginfo("Grasp offsets: " + str(grasp_attempt_offsets))
                transformed_pose = self._tf_listener.transformPose(
                                                                'grasp',
                                                                grasp["grasp"])
                transformed_pre_grasp =  self._tf_listener.transformPose(
                                                                'grasp',
                                                                grasp["pre_grasp"])
                # Good grasp but too many points in palm
                rospy.loginfo("Good grasp but too many points in the palm.")
                for i in range(grasp_attempts):

                    rospy.loginfo("Backing off attempt {}.".format(idx))
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

                    viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
                    viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')



                    rotated = False
                    if 'rolled' in grasp:
                        rotated = grasp['rolled']

                    grasp_in_bounds = self.check_pose_within_bounds(grasp['grasp'], 
                                                                self.shelf_bottom_height, self.shelf_height, 
                                                                self.shelf_width, self.bin_id, 'base_footprint', rotated)
                    if not grasp_in_bounds:
                        rospy.loginfo("Grasp not in bounds")
                        break

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
            rospy.loginfo("Checking grasp: {}".format(grasp["id"]))
            grasp = self.check_reachable(grasp)
            if grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                rospy.loginfo("Grasp {} reachable".format(grasp['id']))
                reachable_good_grasps.append(grasp)
            else:
                rospy.loginfo("Grasp {} not reachable".format(grasp['id']))

        if not reachable_good_grasps:
            for grasp in good_grasps:
                rospy.loginfo("Making grasp {} reachable".format(grasp["id"]))
                # Move and try to check IK
                rospy.loginfo("Trying to make grasp reachable.")
                grasp = self.get_reachable_grasp(grasp)
                if grasp["grasp_quality"] > self.min_grasp_quality and \
                    grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                    rospy.loginfo("Added grasp {} to reachable, good grasps".format(grasp['id']))
                    reachable_good_grasps.append(grasp)


        return self.sort_grasps(reachable_good_grasps)

    def execute_grasp(self, grasps, item_model):
               
        success_pre_grasp = False
        success_grasp = False
        rospy.loginfo("Executing grasp")
        for grasp in grasps:
            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            viz.publish_gripper(self._im_server, grasp["pre_grasp"], 'pre_grasp_target')

            if self._debug:
                raw_input('(Debug) Press enter to continue >')

            rospy.loginfo("Pre_grasp: {}".format(grasp["pre_grasp"]))
            rospy.loginfo("Grasp: {}".format(grasp["grasp"]))
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


            self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene) 
            rospy.wait_for_service('/get_planning_scene', 10.0) 
            get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene) 
            request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX) 
            response = get_planning_scene(request) 

            acm = response.scene.allowed_collision_matrix 
            if not 'bbox' in acm.default_entry_names: 
                # add button to allowed collision matrix 
                acm.default_entry_names += ['bbox'] 
                acm.default_entry_values += [True] 

                planning_scene_diff = PlanningScene( 
                    is_diff=True, 
                    allowed_collision_matrix=acm) 

                #self._pubPlanningScene.publish(planning_scene_diff) 
                #rospy.sleep(1.0)


            self._move_arm_ik.wait_for_service()

            success_grasp = self._move_arm_ik(grasp["grasp"], MoveArmIkRequest().RIGHT_ARM).success

            #self._moveit_move_arm.wait_for_service()
            #success_grasp = self._moveit_move_arm(grasp["grasp"], 
            #                                        0.01, 0.01, 0, 'right_arm',
            #                                        False).success
 


            viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')
            #success_grasp = True
            if success_grasp:
                rospy.loginfo('Grasp succeeded')
                rospy.loginfo('Close Hand')
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(open_left=False, open_right=False,
                                                   effort=item_model.grasp_effort)
                break
        if success_grasp:
            return True
        else:
            return False


    @handle_service_exceptions(outcomes.GRASP_FAILURE)
    def execute(self, userdata):

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


        if userdata.item_model.allow_finger_collisions:
            self.max_finger_collision_points = 1000

        self._tts.publish('Grasping item')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

        self._cluster = userdata.target_cluster

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
                                                                userdata.target_descriptor.planar_bounding_box.pose)

        rospy.loginfo(
            'Grasping item in bin {} from pose {}'
            .format(userdata.bin_id, base_frame_item_pose)
        )
        if userdata.debug:
            raw_input('(Debug) Press enter to continue >')

        planar_bounding_box = userdata.target_descriptor.planar_bounding_box
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object('bbox')
        #rate = rospy.Rate(1)
        for i in range(10):
            scene.add_box("bbox", planar_bounding_box.pose, (planar_bounding_box.dimensions.x, planar_bounding_box.dimensions.y, planar_bounding_box.dimensions.z))
            #scene.add_box("bbox", planar_bounding_box.pose, (3, 3, 3)) 
            rospy.sleep(0.1)
            #rate.sleep()

        grasping_pairs = self.generate_grasps(planar_bounding_box, userdata.bin_id)
        filtered_grasps = self.filter_grasps(grasping_pairs)
        if len(filtered_grasps) > 0:
            success_grasp = self.execute_grasp(filtered_grasps, userdata.item_model)
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
