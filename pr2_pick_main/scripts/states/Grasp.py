import geometry_msgs.msg import Pose, PoseStamped, Quaternion
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

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id', 'clusters', 'debug']
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

        # Grasp Parameters

        self._pre_grasp_dist = 0.38
        self._grasp_height = 0.03
        self._pre_grasp_height = self._grasp_height + 0.02

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

    def locate_one_item(self, clusters):
        '''
        Locate one items in this shelf based on clusters from perception data.
        If there is more than one cluster (unexpected) use the largest one.
        '''
        cluster_to_use = None
        largest_size = 0
        for cluster in clusters:
            size = cluster.pointcloud.height * cluster.pointcloud.width
            if size > largest_size:
                largest_size = size
                cluster_to_use = cluster

        response = self._find_centroid(cluster_to_use)
        return response.centroid

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
        item_point = self.locate_one_item(userdata.clusters)
        if not item_point.header.frame_id:
            rospy.loginfo('Grasping failed. No clusters.')
            self._tts.publish('No clusters. Giving up.')
            return outcomes.GRASP_FAILURE

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

        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object('shelf')
        # self.add_shelf_mesh_to_scene(scene)

        shelf_height = self._shelf_heights[userdata.bin_id]

        # Center Arm
        """
        rospy.loginfo('Center Arm')
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint';

        if userdata.bin_id > 'F':
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.4665;
            pose.pose.position.z = 0.6905;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;
        elif userdata.bin_id > 'C':
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.3865;
            pose.pose.position.z = 0.6905 + 0.23;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;
        else:
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.3865;
            pose.pose.position.z = 0.6905 + 2 * 0.23;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;

        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose, 0.01, 0.01, 0, 'right_arm')
        """
        dist_to_palm = 0.11
        dist_to_fingertips = 0.24
        attempts = 3

        success_pre_grasp = False

        for i in range(attempts):

            # Pose in front of bin

            rospy.loginfo('Pre-grasp:')
            pose_target = PoseStamped()
            pose_target.header.frame_id = 'base_footprint';

            if userdata.bin_id > 'C':

                rospy.loginfo('Not in the top row')
                pose_target.pose.orientation.w = 1
                pose_target.pose.position.x = self._pre_grasp_dist + 0.01 * i
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                if ((base_frame_item_pose.pose.position.z > (shelf_height + self._grasp_height))
                    and (base_frame_item_pose.pose.position.z < (shelf_height + 0.15))):
                    pose_target.pose.position.z = base_frame_item_pose.pose.position.z
                else:
                    pose_target.pose.position.z = shelf_height + \
                        self._grasp_height + self._pre_grasp_height
                rospy.loginfo('pose x: ' + str(pose_target.pose.position.x) +
                              ', y: ' + str(pose_target.pose.position.y) +
                              ', z: ' + str(pose_target.pose.position.z))
                rospy.loginfo('orientation x: ' + str(pose_target.pose.orientation.x) +
                              ', y: ' + str(pose_target.pose.orientation.y) +
                              ', z: ' + str(pose_target.pose.orientation.z) +
                              ', w: ' + str(pose_target.pose.orientation.w))

                viz.publish_gripper(self._im_server, pose_target, 'grasp_target')
                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(pose_target, 0.001, 0.01, 0, 'right_arm').success
            else:
                #   pre  - Translation: [0.253, -0.277, 1.508]
                # - Rotation: in Quaternion [0.984, -0.013, 0.178, 0.028]
                #             in RPY [3.087, -0.359, -0.017]

                rospy.loginfo('In top row')
                pose_target.pose.orientation.x = 0.984
                pose_target.pose.orientation.y = -0.013
                pose_target.pose.orientation.z = 0.178
                pose_target.pose.orientation.w = 0.028
                pose_target.pose.position.x = 0.243 + 0.01 * i
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                pose_target.pose.position.z = 1.508

                rospy.loginfo('pose x: ' + str(pose_target.pose.position.x) +
                              ', y: ' + str(pose_target.pose.position.y) +
                              ', z: ' + str(pose_target.pose.position.z))
                rospy.loginfo('orientation x: ' + str(pose_target.pose.orientation.x) +
                              ', y: ' + str(pose_target.pose.orientation.y) +
                              ', z: ' + str(pose_target.pose.orientation.z) +
                              ', w: ' + str(pose_target.pose.orientation.w))

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
                rospy.loginfo('Pre-grasp attempt ' + str(i) + ' failed')
                self._tts.publish('Pre-grasp attempt ' + str(i) + ' failed')
                continue

        if not success_pre_grasp:
            return outcomes.GRASP_FAILURE

        success_grasp = False

        grasp_attempts = 20
        grasp_attempt_offset = (dist_to_fingertips - dist_to_palm) / grasp_attempts

        for i in range(grasp_attempts):

            # Move into bin

            rospy.loginfo('Grasp')
            pose_target = PoseStamped()
            pose_target.header.frame_id = 'base_footprint';
            if userdata.bin_id > 'C':

                rospy.loginfo('Not grasping from top row')
                pose_target.pose.orientation.w = 1
                pose_target.pose.position.x = base_frame_item_pose.pose.position.x - dist_to_palm - grasp_attempt_offset * i 
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                if ((base_frame_item_pose.pose.position.z > (shelf_height + self._grasp_height))
                    and (base_frame_item_pose.pose.position.z < (shelf_height + 0.15))):
                    pose_target.pose.position.z = base_frame_item_pose.pose.position.z
                else:
                    pose_target.pose.position.z = shelf_height + self._grasp_height


                rospy.loginfo('pose x: ' + str(pose_target.pose.position.x) +
                              ', y: ' + str(pose_target.pose.position.y) +
                              ', z: ' + str(pose_target.pose.position.z))
                rospy.loginfo('orientation x: ' + str(pose_target.pose.orientation.x) +
                              ', y: ' + str(pose_target.pose.orientation.y) +
                              ', z: ' + str(pose_target.pose.orientation.z) +
                              ', w: ' + str(pose_target.pose.orientation.w))

                viz.publish_gripper(self._im_server, pose_target, 'grasp_target')
                self._moveit_move_arm.wait_for_service()
                success_grasp = self._moveit_move_arm(pose_target, 0.0001, 0.001, 0, 'right_arm').success

            else:
                # real - Translation: [0.431, -0.280, 1.570]
                # - Rotation: in Quaternion [0.996, -0.016, 0.080, 0.027]
                #     in RPY [3.090, -0.162, -0.028]

                rospy.loginfo('Grasping from top row')
                pose_target.pose.orientation.x = 0.996
                pose_target.pose.orientation.y = -0.016
                pose_target.pose.orientation.z = 0.080
                pose_target.pose.orientation.w = 0.027
                pose_target.pose.position.x = 0.431 - 0.01 * i
                pose_target.pose.position.y = base_frame_item_pose.pose.position.y
                pose_target.pose.position.z = 1.570

                rospy.loginfo('Grasping from top row')

                rospy.loginfo('pose x: ' + str(pose_target.pose.position.x) +
                              ', y: ' + str(pose_target.pose.position.y) +
                              ', z: ' + str(pose_target.pose.position.z))
                rospy.loginfo('orientation x: ' + str(pose_target.pose.orientation.x) +
                              ', y: ' + str(pose_target.pose.orientation.y) +
                              ', z: ' + str(pose_target.pose.orientation.z) +
                              ', w: ' + str(pose_target.pose.orientation.w))

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
                rospy.loginfo('Grasp attempt '  + str(i) + ' failed')
                self._tts.publish('Grasp attempt ' + str(i) + ' failed')
                continue

        if not success_grasp:
            rospy.loginfo('Grasping failed')
            self._tts.publish('Grasping failed. Giving up.')
            return outcomes.GRASP_FAILURE
        else:
            rospy.loginfo('Grasping succeeded')
            self._tts.publish('Grasping succeeded.')
            return outcomes.GRASP_SUCCESS
