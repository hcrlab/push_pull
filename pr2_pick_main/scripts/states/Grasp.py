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
from pr2_pick_manipulation.srv import GetPose, MoveArm, SetGrippers
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest

""" Temporary class for moving arm without collision checking until we have service """

class Arm:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                        JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move(self, waypoints):
        goal = JointTrajectoryGoal()
        char = self.name[0] #either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                   char+'_shoulder_lift_joint',
                   char+'_upper_arm_roll_joint',
                   char+'_elbow_flex_joint',
                   char+'_forearm_roll_joint',
                   char+'_wrist_flex_joint',
                   char+'_wrist_roll_joint']
        time_offset = 0
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        rospy.loginfo('Sending joint goal')
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = rospy.Duration(1.5 + time_offset*1.5)
            goal.trajectory.points.append(point)
            time_offset = time_offset + 1
            
        self.jta.send_goal_and_wait(goal)


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
    min_points_in_gripper = 90

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
        self._set_static_tf = services['set_static_tf']

        self._fk_client = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self._ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._get_points_in_box = rospy.ServiceProxy('perception/get_points_in_box', BoxPoints)

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

    def grasp_msg_to_poses(self, grasp_msgs):
        grasping_pairs = []

        # get rid of every second message because they're just repeats in different frames
        grasp_msgs = grasp_msgs[::2]

        for grasp in grasp_msgs:
            if not grasp.grasp_quality >= self.min_grasp_quality:
                continue

            # rospy.loginfo("Frame_id: " + str(grasp.pre_grasp_approach.direction.header.frame_id))
            # rospy.loginfo("Vector: {}, {}, {}"
            #                 .format(grasp.pre_grasp_approach.direction.vector.x, 
            #                     grasp.pre_grasp_approach.direction.vector.y, 
            #                     grasp.pre_grasp_approach.direction.vector.z))
            # rospy.loginfo("Desirec dist: " + str(grasp.pre_grasp_approach.desired_distance))
            # rospy.loginfo("Min dist: " + str(grasp.pre_grasp_approach.min_distance))
            # position = grasp.grasp_pose.pose.position
            # rospy.loginfo(
            #     'pose x: {}, y: {}, z: {}'
            #     .format(position.x, position.y, position.z)
            # )
            # rospy.loginfo("Grasp Frame_id: " + str(grasp.grasp_pose.header.frame_id))

            # rospy.loginfo("Number of pre_grasp_posture points: " + str(len(grasp.pre_grasp_posture.points)))


            
            fk_request = GetPositionFKRequest()
            fk_request.header.frame_id = "base_footprint"
            fk_request.fk_link_names = ["r_wrist_roll_link"]
            fk_request.robot_state.joint_state.position = grasp.pre_grasp_posture.points[0].positions
            fk_request.robot_state.joint_state.name = grasp.pre_grasp_posture.joint_names
            fk_response = self._fk_client(fk_request)

            # rospy.loginfo("FK position: {}, {}, {}".format(fk_response.pose_stamped[0].pose.position.x,
            #                                                 fk_response.pose_stamped[0].pose.position.y,
            #                                                 fk_response.pose_stamped[0].pose.position.z))

            grasp_pose = self._tf_listener.transformPose('base_footprint',
                                                                grasp.grasp_pose)
            pre_grasp_pose = fk_response.pose_stamped[0]

            rospy.loginfo(" ")
            rospy.loginfo(" ")

            grasp_dict = {}
            grasp_dict["pre_grasp"] = pre_grasp_pose
            grasp_dict["grasp"] = grasp_pose
            grasping_pairs.append(grasp_dict)

        return grasping_pairs

    def generate_grasps(self, object_pose, bin_id):

        grasping_pairs = [] # list of pre_grasp/grasp dicts 

        shelf_height = self._shelf_heights[bin_id]

        # Pre-grasp: pose arm in front of bin

        pre_grasp_pose_target = PoseStamped()
        pre_grasp_pose_target.header.frame_id = 'base_footprint';

        if bin_id > 'C':
            rospy.loginfo('Not in the top row')
            pre_grasp_pose_target.pose.orientation.w = 1
            pre_grasp_pose_target.pose.position.x = self.pre_grasp_x_distance 
            pre_grasp_pose_target.pose.position.y = object_pose.pose.position.y

            # go for centroid if it's vertically inside shelf
            if ((object_pose.pose.position.z > (shelf_height + self.half_gripper_height))
                and (object_pose.pose.position.z < (shelf_height + 0.15))):
                pre_grasp_pose_target.pose.position.z = object_pose.pose.position.z
            # otherwise, centroid is probably wrong, just use lowest possible grasp
            else:
                pre_grasp_pose_target.pose.position.z = shelf_height + \
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
            if ((object_pose.pose.position.z > (shelf_height + self.half_gripper_height))
                and (object_pose.pose.position.z < (shelf_height + 0.15))):
                grasp_pose_target.pose.position.z = object_pose.pose.position.z
            else:
                grasp_pose_target.pose.position.z = shelf_height + self.half_gripper_height


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
        goal.pose = object_pose.pose
        goal.width = 0.0

        options_1 = GraspGeneratorOptions()
        options_1.grasp_axis = options_1.GRASP_AXIS_Y
        options_1.grasp_direction = options_1.GRASP_DIRECTION_UP
        options_1.grasp_rotation = options_1.GRASP_ROTATION_FULL

        options_2 = GraspGeneratorOptions()
        options_2.grasp_axis = options_2.GRASP_AXIS_Y
        options_2.grasp_direction = options_2.GRASP_DIRECTION_UP
        options_2.grasp_rotation = options_2.GRASP_ROTATION_FULL

        goal.options.append(options_1)
        goal.options.append(options_2)
        grasp_action_client.send_goal(goal)
        grasp_action_client.wait_for_result()

        grasps_result = grasp_action_client.get_result()

        moveit_simple_grasps  = self.grasp_msg_to_poses(grasps_result.grasps)

        # Commented out for testing purposes
        #grasping_pairs = grasping_pairs + moveit_simple_grasps

        return grasping_pairs

    def evaluate_grasp(self, grasp):
        # just a placeholder, returns dict [pre_grasp, grasp, pre_grasp_reachable, grasp_reachable, grasp_quality]
        # may want to make a class
        viz.publish_gripper(self._im_server, grasp["grasp"], 'grasp_target')

        # Check if enough points will be in gripper
        # TODO: Check if too many points will be inside fingers
        # TODO: Publish marker for bounding box of points
        rospy.loginfo("Just checking grasp quality") 

        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = grasp["grasp"].pose.position
        transform.transform.rotation = grasp["grasp"].pose.orientation
        transform.child_frame_id = 'grasp'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        rospy.sleep(2.0)

        box_request = BoxPointsRequest()
        box_request.cluster = self._cluster
        box_request.frame_id = "grasp"
        box_request.min_x = self.dist_to_palm
        box_request.max_x = self.dist_to_fingertips
        box_request.min_y = -1 * self.gripper_palm_width/2
        box_request.max_y = self.gripper_palm_width/2
        box_request.min_z = -1 * self.gripper_finger_height/2
        box_request.max_z = self.gripper_finger_height/2
        self._get_points_in_box.wait_for_service()
        box_response = self._get_points_in_box(box_request)

        rospy.loginfo("Number of points inside gripper: {}".format(box_response.num_points))

        if box_response.num_points >= self.min_points_in_gripper:
            grasp["grasp_quality"] = 1.0
        else:
            grasp["grasp_quality"] = 0.0

        # Only check reachability if not already checked
        if not "grasp_reachable" in grasp.keys():
            rospy.loginfo("Full grasp evaluation")
            grasp["grasp_quality"] = 1.0
            # Check pre-grasp IK
            rospy.loginfo("Checking pre-grasp ik")
            ik_request = GetPositionIKRequest()
            ik_request.ik_request.group_name = "right_arm"
            ik_request.ik_request.pose_stamped = grasp["pre_grasp"]
            ik_response = self._ik_client(ik_request)

            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                grasp["pre_grasp_reachable"] = True
            else: 
                grasp["pre_grasp_reachable"] = False

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

    def filter_grasps(self, grasps):
        filtered_grasps = []
        for grasp in grasps:
            # Check if both pre-grasp and grasp are reachable
            if grasp["pre_grasp_reachable"] and grasp["grasp_reachable"]:
                filtered_grasps.append(grasp)
        return filtered_grasps

    def get_reachable_grasps(self, grasps):
        reachable_grasps = []
        for grasp in grasps:
            # If pre-grasp not reachable, try
            if not grasp["pre_grasp_reachable"]:
                pre_grasp_offsets = [
                    self.pre_grasp_attempt_separation * i
                    for i in range(self.pre_grasp_attempts)
                ]
                reachable = False
                for (idx, offset) in enumerate(pre_grasp_offsets):
                    # transform pre-grasp into r_wrist_roll_link frame
                    transformed_pose = self._tf_listener.transformPose('r_wrist_roll_link',
                                                                grasp["pre_grasp"])
                    # move it forward in x
                    transformed_pose.pose.position.x = transformed_pose.pose.position.x + offset

                    #check ik
                    rospy.loginfo("Checking ik")
                    ik_request = GetPositionIKRequest()
                    ik_request.ik_request.group_name = "right_arm"
                    ik_request.ik_request.pose_stamped = transformed_pose
                    ik_response = self._ik_client(ik_request)

                    #if reachable, set True, set new pre-grasp, break
                    if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                        reachable = True
                        pose_in_base_footprint = self._tf_listener.transformPose('base_footprint',
                                                                transformed_pose)
                        grasp["pre_grasp"] = pose_in_base_footprint
                        grasp["pre_grasp_reachable"] = True
                        break

                if not reachable:
                    continue 

            if not grasp["grasp_reachable"]:
                grasp_attempt_delta = (self.dist_to_fingertips - self.dist_to_palm) / self.grasp_attempts
                grasp_attempt_offsets = [
                    grasp_attempt_delta * i
                    for i in range(self.grasp_attempts)
                ]
                for (idx, offset) in enumerate(grasp_attempt_offsets):
                    # transform grasp into r_wrist_roll_link frame
                    transformed_pose = self._tf_listener.transformPose('r_wrist_roll_link',
                                                                grasp["grasp"])

                    # move it forward in x
                    transformed_pose.pose.position.x = transformed_pose.pose.position.x + offset

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
                        evaluated_grasp = self.evaluate_grasp(grasp)

                        # If object in gripper, keep 
                        if evaluated_grasp["grasp_quality"] > self.min_grasp_quality:
                            reachable_grasps.append(grasp)
                        break

        return reachable_grasps

    def evaluate_grasps(self, grasps):
        evaluated_grasps = []
        num_reachable = 0
        attempts = 0
        
        for grasp in grasps:
            evaluated_grasp = self.evaluate_grasp(grasp)
            if evaluated_grasp["grasp_quality"] > 0:
                evaluated_grasps.append(evaluated_grasp)
                if evaluated_grasp["pre_grasp_reachable"] and evaluated_grasp["grasp_reachable"]:
                    num_reachable = num_reachable + 1

        if num_reachable == 0:
            evaluated_grasps = self.get_reachable_grasps(evaluated_grasps)
        else:
            evaluated_grasps = self.filter_grasps(evaluated_grasps)

        return evaluated_grasps

    def execute_grasp(self, grasps):
        success_pre_grasp = False
        success_grasp = False
        for grasp in grasps:
            self._moveit_move_arm.wait_for_service()
            success_pre_grasp = self._moveit_move_arm(grasp["pre_grasp"], 0.01, 0.01, 0, 'right_arm').success
 
            if not success_pre_grasp:
                continue
            else:
                rospy.loginfo('Pre-grasp succeeeded')
                rospy.loginfo('Open Hand')
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(False, True)
            #self._moveit_move_arm.wait_for_service()
            #success_grasp = self._moveit_move_arm(grasp["grasp"], 0.01, 0.01, 0, 'right_arm').success
            # Temporary hack for before we have an IK service

            ik_request = GetPositionIKRequest()
            ik_request.ik_request.group_name = "right_arm"
            ik_request.ik_request.pose_stamped = grasp["grasp"]
            char = 'r'
            ik_request.ik_request.robot_state.joint_state.name = [char+'_shoulder_pan_joint',
                   char+'_shoulder_lift_joint',
                   char+'_upper_arm_roll_joint',
                   char+'_elbow_flex_joint',
                   char+'_forearm_roll_joint',
                   char+'_wrist_flex_joint',
                   char+'_wrist_roll_joint']
            ik_response = self._ik_client(ik_request)
            arm = Arm('r_arm')
            rospy.loginfo("Joint positions: " + str(ik_response.solution.joint_state.position))
            rospy.loginfo("Joint names: " + str(ik_response.solution.joint_state.name))
            positions = [None] * 7
            for (ind, name) in enumerate(ik_response.solution.joint_state.name):
                if name in ik_request.ik_request.robot_state.joint_state.name:
                    idx = ik_request.ik_request.robot_state.joint_state.name.index(name)
                    positions[idx] = ik_response.solution.joint_state.position[ind]
                
            arm.move([positions])
            success_grasp = True
            if success_grasp:
                rospy.loginfo('Grasp succeeded')
                rospy.loginfo('Close Hand')
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(False, False)
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

        # scene = moveit_commander.PlanningSceneInterface()
        # scene.remove_world_object('shelf')
        # self.add_shelf_mesh_to_scene(scene)

        grasping_pairs = self.generate_grasps(base_frame_item_pose, userdata.bin_id)
        evaluated_grasps = self.evaluate_grasps(grasping_pairs)
        if len(evaluated_grasps) > 0:
            success_grasp = self.execute_grasp(evaluated_grasps)
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
