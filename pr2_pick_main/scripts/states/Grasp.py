from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import json
import moveit_commander
import os
import rospkg
import rospy
import smach
from std_msgs.msg import Header
import tf
import visualization as viz

import outcomes
from pr2_pick_manipulation.srv import GetPose, MoveArm, SetGrippers


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
    pre_grasp_x_distance = 0.38

    # approximate distance from center to edge of gripper pad
    half_gripper_height = 0.03
    # approximate distance from palm frame origin to palm surface
    dist_to_palm = 0.11
    # approximate distance from palm frame origin to fingertip with gripper closed
    dist_to_fingertips = 0.24

    pre_grasp_height = half_gripper_height + 0.02

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id', 'debug', 'target_cluster']
        )

        self._find_centroid = services['find_centroid']
        self._set_grippers = services['set_grippers']
        self._tuck_arms = services['tuck_arms']
        self._moveit_move_arm = services['moveit_move_arm']
        self._tts = services['tts']
        self._tf_listener = services['tf_listener']
        self._im_server = services['interactive_marker_server']

        self._wait_for_transform_duration = rospy.Duration(5.0)

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

    def add_shelf_mesh_to_scene(scene):
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

    def execute(self, userdata):
        self._tts.publish('Grasping item')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

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

        shelf_height = self._shelf_heights[userdata.bin_id]

        # Pre-grasp: pose arm in front of bin
        success_pre_grasp = False
        pre_grasp_offsets = [
            self.pre_grasp_attempt_separation * i
            for i in range(self.pre_grasp_attempts)
        ]
        for (idx, offset) in enumerate(pre_grasp_offsets):

            rospy.loginfo('Pre-grasp:')
            pose_target = PoseStamped()
            pose_target.header.frame_id = 'base_footprint';

            if userdata.bin_id > 'C':
                rospy.loginfo('Not in the top row')
                pose_target.pose.orientation.w = 1
                pose_target.pose.position.x = self.pre_grasp_x_distance + offset
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y

                # go for centroid if it's vertically inside shelf
                if ((base_frame_item_pose.pose.position.z > (shelf_height + self.half_gripper_height))
                    and (base_frame_item_pose.pose.position.z < (shelf_height + 0.15))):
                    pose_target.pose.position.z = base_frame_item_pose.pose.position.z
                # otherwise, centroid is probably wrong, just use lowest possible grasp
                else:
                    pose_target.pose.position.z = shelf_height + \
                        self.half_gripper_height + self.pre_grasp_height

                self.log_pose_info(pose_target.pose)

                viz.publish_gripper(self._im_server, pose_target, 'grasp_target')
                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(pose_target, 0.001, 0.01, 0, 'right_arm').success
            else:
                rospy.loginfo('In top row')
                pose_target.pose.orientation.x = 0.984
                pose_target.pose.orientation.y = -0.013
                pose_target.pose.orientation.z = 0.178
                pose_target.pose.orientation.w = 0.028
                pose_target.pose.position.x = 0.243 + offset
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                pose_target.pose.position.z = 1.508

                self.log_pose_info(pose_target.pose)

                viz.publish_gripper(self._im_server, pose_target, 'grasp_target')
                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(pose_target, 0.01, 0.01, 0, 'right_arm').success
                rospy.loginfo('Worked: ' + str(success_pre_grasp))

            if success_pre_grasp:
                # Open Hand
                rospy.loginfo('Pre-grasp succeeeded')
                rospy.loginfo('Open Hand')
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(False, True)
                break
            else:
                rospy.loginfo('Pre-grasp attempt ' + str(idx) + ' failed')
                self._tts.publish('Pre-grasp attempt ' + str(idx) + ' failed')
                continue

        if not success_pre_grasp:
            return outcomes.GRASP_FAILURE

        success_grasp = False

        # Move gripper into bin
        grasp_attempt_delta = (self.dist_to_fingertips - self.dist_to_palm) / self.grasp_attempts
        grasp_attempt_offsets = [
            grasp_attempt_delta * i
            for i in range(self.grasp_attempts)
        ]
        for (idx, offset) in enumerate(grasp_attempt_offsets):
            rospy.loginfo('Grasp')
            pose_target = PoseStamped()
            pose_target.header.frame_id = 'base_footprint';
            if userdata.bin_id > 'C':

                rospy.loginfo('Not grasping from top row')
                pose_target.pose.orientation.w = 1
                pose_target.pose.position.x = \
                    base_frame_item_pose.pose.position.x - self.dist_to_palm - offset
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                if ((base_frame_item_pose.pose.position.z > (shelf_height + self.half_gripper_height))
                    and (base_frame_item_pose.pose.position.z < (shelf_height + 0.15))):
                    pose_target.pose.position.z = base_frame_item_pose.pose.position.z
                else:
                    pose_target.pose.position.z = shelf_height + self.half_gripper_height

                self.log_pose_info(pose_target.pose)

                viz.publish_gripper(self._im_server, pose_target, 'grasp_target')
                self._moveit_move_arm.wait_for_service()
                success_grasp = self._moveit_move_arm(pose_target, 0.0001, 0.001, 0, 'right_arm').success

            else:
                rospy.loginfo('Grasping from top row')
                pose_target.pose.orientation.x = 0.996
                pose_target.pose.orientation.y = -0.016
                pose_target.pose.orientation.z = 0.080
                pose_target.pose.orientation.w = 0.027
                pose_target.pose.position.x = 0.431 - offset
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                pose_target.pose.position.z = 1.570

                rospy.loginfo('Grasping from top row')
                self.log_pose_info(pose_target.pose)

                viz.publish_gripper(self._im_server, pose_target, 'grasp_target')
                self._moveit_move_arm.wait_for_service()
                success_grasp = self._moveit_move_arm(pose_target, 0.01, 0.01, 0, 'right_arm').success

            if success_grasp:
                # Close hand
                rospy.loginfo('Grasp succeeded')
                rospy.loginfo('Close Hand')
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(False, False)

                break
            else:
                rospy.loginfo('Grasp attempt '  + str(idx) + ' failed')
                self._tts.publish('Grasp attempt ' + str(idx) + ' failed')
                continue

        if not success_grasp:
            rospy.loginfo('Grasping failed')
            self._tts.publish('Grasping failed. Giving up.')
            return outcomes.GRASP_FAILURE
        else:
            rospy.loginfo('Grasping succeeded')
            self._tts.publish('Grasping succeeded.')
            return outcomes.GRASP_SUCCESS
