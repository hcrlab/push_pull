
from actionlib import SimpleActionClient
from pr2_pick_main import handle_service_exceptions
from pr2_pick_manipulation.srv import TuckArms
from pr2_pick_manipulation.srv import MoveHead
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
import outcomes
import rospy
import smach
import tf
import visualization as viz
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import moveit_commander
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal

class StartPoseExperiment(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'START_POSE'

    def __init__(self, tts, tuck_arms, move_head, tf_listener, **kwargs):
        """Constructor for this state.
        Args:
          tts: A publisher for the TTS node
          tuck_arms: The tuck arms service proxy.
          move_head: The head service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.START_POSE_SUCCESS, outcomes.START_POSE_FAILURE],
            input_keys=['debug'],
            output_keys=['start_pose'])
        # approximate tool dimensions
        self.tool_x_size = 0.26
        self.tool_y_size = 0.01
        self.tool_z_size = 0.03
        # tool position relative to wrist_roll_link
        self.tool_x_pos = 0.29
        self.tool_y_pos = 0.0
        self.tool_z_pos = 0.0
        self.arm_side = 'l'
        self.tool_name = 'tool'
        self.waypoint_duration = rospy.Duration(10.0)


        self._tts = tts
        self._tuck_arms = tuck_arms
        self._move_head = move_head
        self._tf_listener = tf_listener
        self._markers = kwargs['markers']
        self._drive_to_pose = kwargs['drive_to_pose']
        self._im_server = kwargs['interactive_marker_server']
        self._moveit_move_arm = kwargs['moveit_move_arm']
        # The location the robot started at.
        # When the robot relocalizes, it goes back to this start pose.
        self._start_pose = None
        self._move_torso = kwargs['move_torso']
        self._set_static_tf = kwargs['set_static_tf']
        self._set_grippers = kwargs['set_grippers']
        self._get_grippers = kwargs['get_grippers']
        self._attached_collision_objects = kwargs['attached_collision_objects']
        self._get_planning_scene = kwargs['get_planning_scene']
        self.moveit_object_name = 'push_item_target_bbox'
        self._planning_scene_publisher = kwargs['planning_scene_publisher']
        self._interactive_markers = kwargs['interactive_marker_server']
        self.arm = SimpleActionClient(
            '{}_arm_controller/joint_trajectory_action'.format(self.arm_side),
            JointTrajectoryAction,
        )
    def _adjust_start_pose_orientation(self):
        # After driving around enough, odom_combined seems to have a lot of
        # rotation error. Orient ourselves so we're facing the shelf.
        try:
            shelf_in_shelf = PoseStamped()
            shelf_in_shelf.header.frame_id = 'shelf'
            shelf_in_shelf.header.stamp = rospy.Time(0)
            shelf_in_shelf.pose.orientation.w = 1
            shelf_in_odom = self._tf_listener.transformPose('odom_combined',
                                                            shelf_in_shelf)
            self._start_pose.pose.orientation = shelf_in_odom.pose.orientation
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr('No transform between {} and {} in FindShelf'.format(
                'shelf', self._start_pose.header.frame_id))

    def _drive_to_start_pose(self):
        viz.publish_base(self._markers, self._start_pose)
        self._drive_to_pose.wait_for_service()
        self._drive_to_pose(self._start_pose, 0.1, 0.1)

    # Pre-move_object position
    def pre_position_tool(self):
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

          success_pre_grasp = self._moveit_move_arm(pre_grasp_pose, 
                                                  0.005, 0.005, 12, 'left_arm',
                                                  False).success

    def add_tool_collision_object(self):
        '''
        Add an attached collision object representing the tool to the moveit
        planning scene so moveit can plan knowing that the tool is solid and
        moves with the robot's gripper.
        '''

        box = SolidPrimitive(type=SolidPrimitive.BOX)
        box.dimensions = [self.tool_x_size, self.tool_y_size, self.tool_z_size]
        box_pose = Pose()
        box_pose.position.x = self.tool_x_pos
        box_pose.position.y = self.tool_y_pos
        box_pose.position.z = self.tool_z_pos

        collision_object = CollisionObject()
        collision_object.header.frame_id = '{}_wrist_roll_link'.format(self.arm_side)
        collision_object.id = self.tool_name
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives = [box]
        collision_object.primitive_poses = [box_pose]

        tool = AttachedCollisionObject()
        tool.link_name = '{}_wrist_roll_link'.format(self.arm_side)
        joints_below_forearm = [
            '{}_forearm_link', '{}_wrist_flex_link', '{}_wrist_roll_link',
            '{}_gripper_palm_link', '{}_gripper_l_finger_link',
            '{}_gripper_l_finger_tip_link', '{}_gripper_motor_accelerometer_link',
            '{}_gripper_r_finger_link', '{}_gripper_r_finger_tip_link',
        ]
        tool.touch_links = [
            link.format(self.arm_side)
            for link in joints_below_forearm
        ]
        tool.object = collision_object

        for i in range(10):
            self._attached_collision_objects.publish(tool)

        pose_stamped = PoseStamped(header=collision_object.header, pose=box_pose)

        viz.publish_bounding_box(
            self._interactive_markers, pose_stamped,
            self.tool_x_size, self.tool_y_size, self.tool_z_size,
            0.5, 0.2, 0.1, 0.9,
            2,
        )

    @handle_service_exceptions(outcomes.START_POSE_FAILURE)
    def execute(self, userdata):
        
        # Publish static transform.
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        odom = PoseStamped()
        odom.pose.position.x = 0.0
        odom.pose.position.y = 0.0
        odom.pose.position.z = 0.0
        odom.pose.orientation.x = 0.0
        odom.pose.orientation.y = 0.0
        odom.pose.orientation.z = 0.0
        odom.pose.orientation.w = 1.0

        transform.transform.translation = odom.pose.position
        transform.transform.rotation = odom.pose.orientation
        transform.child_frame_id = 'odom_combined'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)        

        rospy.loginfo('Setting start pose.')
        self._tts.publish('Setting start pose.')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
        
        tuck_success = True
        if not tuck_success:
            rospy.logerr('StartPose: TuckArms failed')
            self._tts.publish('Failed to tuck arms.')
        else:
            rospy.loginfo('StartPose: TuckArms success')

        #self.pre_position_tool()
        grippers_open = self._set_grippers(open_left=True, open_right=True, effort =-1)
        raw_input("Press Enter and add tool to the robot ")
        rospy.sleep(4)
        rospy.loginfo("Waiting for set grippers service")
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(open_left=False, open_right=False, effort=-1)
        
        gripper_states = self._get_grippers()
        if not gripper_states.left_open:
            self._set_grippers(open_left=False, open_right=False, effort=-1)

        self.add_tool_collision_object()

        #self._tuck_arms.wait_for_service()
        #tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
        

        if self._start_pose is None:
            try:
                here = PoseStamped()
                here.header.frame_id = 'base_footprint'
                here.header.stamp = rospy.Time(0)
                here.pose.position.x = 0
                here.pose.position.y = 0
                here.pose.position.z = 0
                here.pose.orientation.w = 1
                here.pose.orientation.x = 0
                here.pose.orientation.y = 0
                here.pose.orientation.z = 0
                self._start_pose = self._tf_listener.transformPose(
                    'odom_combined', here)
                userdata.start_pose = self._start_pose
            except:
                rospy.logerr('No transform for start pose.')
                return outcomes.START_POSE_FAILURE
        else:
            self._adjust_start_pose_orientation()
            self._drive_to_start_pose()
        #grippers_open = self._set_grippers(True, True, -1)
        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        if tuck_success:
            return outcomes.START_POSE_SUCCESS
        else:
            self._tts.publish('Start pose failed.')
            return outcomes.START_POSE_FAILURE
