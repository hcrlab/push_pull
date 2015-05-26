from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_pick_main import handle_service_exceptions
from std_msgs.msg import Header
from pr2_pick_manipulation.srv import MoveArmIk, MoveArmIkRequest
from visualization_msgs.msg import Marker
import moveit_commander
import outcomes
import rospy
import smach
import tf
import visualization as viz

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
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = rospy.Duration(1 + time_offset)
            goal.trajectory.points.append(point)
            time_offset = time_offset + 1
            
        self.jta.send_goal_and_wait(goal)


class DropOffItem(smach.State): 
    """Deposits the item into the order bin.
    """
    name = 'DROP_OFF_ITEM'

    # The x,y coordinates the base should drive to for dropoffs in the order
    # bin frame
    DROPOFF_POS_BASE_X = -0.5040 
    DROPOFF_POS_BASE_Y = 0.6604
    # The position the arm will move to before it lets go of the object
    DROPOFF_POS_ARM_X = 0.0872
    DROPOFF_POS_ARM_Y = -0.8277
    DROPOFF_POS_ARM_Z = 0.6577
    DROPOFF_QUAT_ARM_X = 0.0008
    DROPOFF_QUAT_ARM_Y = -0.7025
    DROPOFF_QUAT_ARM_Z = 0.0197
    DROPOFF_QUAT_ARM_W = -0.7114
    # The height the arm will start at before lowering into the bin to dropoff
    # object
    DROPOFF_POS_ARM_START_Z = 0.7477


    above_joint_positions_a_c = [-0.35011491317222343, -1.4795590541938846, 0.9204268572072701, 122.73570699916009, -0.7914346859720043, -1.3839844110051667, 115.03540064173117]
    lower_joint_positions_a_c = [-0.2752292092574864, -1.3794907670440026, 0.7549591876149567, 122.77197712267808, -0.15386447303894846, -0.9296642673251947, 114.93219748325492]
    above_joint_positions_d_f = [-0.35011491317222343, -1.5068353313043328, 0.9183965790527632, 122.73397158655156, -1.0595500434542977, -1.6479965381695019, 115.0646386360381]
    lower_joint_positions_d_f = [-0.35011491317222343, -1.477071855673175, 0.917973604437241, 122.7370953292469, -0.7711667863783103, -1.3662327716045377, 115.0333992314066]
    above_joint_positions_g_i = [-0.35011491317222343, -1.5216756158112332, 0.8228889108678339, 122.7795550910686, -1.2851028117898338, -1.9574754868379154, 115.11210686786677]
    lower_joint_positions_g_i = [-0.35011491317222343, -1.5123071680498938, 0.9034232776632748, 122.74062400155087, -1.1294742970525415, -1.7306634863587442, 115.07490674118162]
    above_joint_positions_j_l = [-0.012728358489982883, -1.3654795487106721, 0.8493671217995282, 122.53752287926746, -1.2919070352248598, -1.9821450445343922, 114.91627321849847]
    lower_joint_positions_j_l = [-0.35011491317222343, -1.5227534018368742, 0.8009788257837802, 122.790777425937, -1.3078318134770477, -1.997634220089851, 115.12120023216758]



    def __init__(self, **kwargs):
        smach.State.__init__(self,
            outcomes=[
                outcomes.DROP_OFF_ITEM_SUCCESS,
                outcomes.DROP_OFF_ITEM_FAILURE
            ],
            input_keys=['bin_id', 'bin_data'],
            output_keys=['output_bin_data']
        )
        self._tts = kwargs["tts"]
        self._set_grippers = kwargs["set_grippers"]
        self._drive_linear = kwargs["drive_linear"]
        self._moveit_move_arm = kwargs["moveit_move_arm"]
        self._move_arm_ik = kwargs["move_arm_ik"]
        self._tuck_arms = kwargs["tuck_arms"]
        self._set_static_tf = kwargs["set_static_tf"]
        self._markers = kwargs["markers"]
        self._drive_to_pose = kwargs["drive_to_pose"]
        self._tf_listener = kwargs["tf_listener"]

        self._order_bin_found = False

        self._arm = Arm("r_arm")

    def get_position(self, base_frame="odom_combined"):
        self._tf_listener.waitForTransform(base_frame,"base_footprint",rospy.Time(0), 
                                           rospy.Duration(1.0))
        (pos,orient) = self._tf_listener.lookupTransform(base_frame,"base_footprint", 
                                                         rospy.Time(0))
        return pos

    def planningscene_create_box(self, position, size, name):
        """Create a box in the MoveIt planning scene specified by position and size, both tuples"""
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object(name)
        table_pose = PoseStamped()
        table_pose.header.frame_id = "odom_combined"
        table_pose.pose.position.x = position[0]
        table_pose.pose.position.y = position[1]
        table_pose.pose.position.z = position[2]
        for i in range(10):
            rospy.sleep(0.10)
            scene.add_box(name, table_pose, size)

    @handle_service_exceptions(outcomes.DROP_OFF_ITEM_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Dropping off item from bin {}'.format(userdata.bin_id))
        self._tts.publish('Dropping off item from bin {}'.format(userdata.bin_id))

        if userdata.bin_id < D:
            self._above_bin_joint_positions = above_joint_positions_a_c
            self._lower_arm_joint_positions = lower_joint_positions_a_c
        elif userdata.bin_id < G:
            self._above_bin_joint_positions = above_joint_positions_d_f
            self._lower_arm_joint_positions = lower_joint_positions_d_f
        elif userdata.bin_id < J:
            self._above_bin_joint_positions = above_joint_positions_g_i
            self._lower_arm_joint_positions = lower_joint_positions_g_i
        else:
            self._above_bin_joint_positions = above_joint_positions_j_l
            self._lower_arm_joint_positions = lower_joint_positions_j_l
        

        if not self._order_bin_found:
            rospy.loginfo('Creating a static TF for the order bin relative to the shelf')


            # Do it the simple way! Assumes the shelf is not too tilted
            # TODO(jstn): move this to FindShelf.
            # Creates a stransform from the shelf frame to the order bin fame
            order_bin_tf = TransformStamped()
            order_bin_tf.header.frame_id = 'shelf'
            order_bin_tf.header.stamp = rospy.Time.now()
            order_bin_tf.transform.translation.x = -36 * 0.0254
            order_bin_tf.transform.translation.y = -27 * 0.0254 # -27
            order_bin_tf.transform.translation.z = 12 * 0.0254
            order_bin_tf.transform.rotation.w = 1
            order_bin_tf.child_frame_id = 'order_bin'
            self._set_static_tf.wait_for_service()
            self._set_static_tf(order_bin_tf)

            self._order_bin_found = True

        # TODO(jstn): move this to FindShelf.
        viz.publish_order_bin(self._markers)


        #rospy.loginfo('Untucking right arm')
        #self._tuck_arms.wait_for_service()
        #tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)

        self._tf_listener.waitForTransform('order_bin',"shelf",rospy.Time(0),
                                           rospy.Duration(5.0))

        # move to the order bin
        rospy.loginfo('Move next to the order bin')

        target_in_order_bin_frame = PoseStamped(
            header=Header(frame_id='order_bin'),
            pose=Pose(
                position=Point(x=self.DROPOFF_POS_BASE_X,
                               y=self.DROPOFF_POS_BASE_Y,
                               z=0.0),
                orientation=Quaternion(w=1, x=0, y=0, z=0)
            )
        )

        # Visualize target pose.
        viz.publish_base(self._markers, target_in_order_bin_frame)

        rospy.loginfo('Sending drive command')
        self._drive_to_pose.wait_for_service()
        move_success = self._drive_to_pose(pose=target_in_order_bin_frame, linearVelocity=0.1, angularVelocity=0.1)
        rospy.loginfo(move_success)

        # move arm above bin
        rospy.loginfo('Move arm above order bin')
        """
        pose_target = PoseStamped()
        pose_target.header.frame_id = "base_footprint";
        pose_target.pose.position.x = self.DROPOFF_POS_ARM_X
        pose_target.pose.position.y = self.DROPOFF_POS_ARM_Y
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_START_Z
        pose_target.pose.orientation.x = self.DROPOFF_QUAT_ARM_X
        pose_target.pose.orientation.y = self.DROPOFF_QUAT_ARM_Y
        pose_target.pose.orientation.z = self.DROPOFF_QUAT_ARM_Z
        pose_target.pose.orientation.w = self.DROPOFF_QUAT_ARM_W 
        self._move_arm_ik.wait_for_service()
        arm_above_bin_success = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM)
        rospy.loginfo(arm_above_bin_success)
        """
        
        waypoints = [self._above_bin_joint_positions]
        self._arm.move(waypoints)

        # lower arm into bin
        rospy.loginfo('Move arm above order bin')
        """
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_Z 
        self._move_arm_ik.wait_for_service()
        arm_into_bin_success = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM)
        rospy.loginfo(arm_into_bin_success)
        """

        waypoints = [self._lower_arm_joint_positions]
        self._arm.move(waypoints)


        # open gripper
        rospy.loginfo('Open gripper')
        self._set_grippers.wait_for_service()
        open_gripper_success = self._set_grippers(False, True, -1)
        rospy.loginfo(open_gripper_success)

        # raise arm
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_START_Z
        self._move_arm_ik.wait_for_service()
        arm_out_of_bin_success = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM)
        rospy.loginfo(arm_out_of_bin_success)

        # get back to "untucked" position
        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        retucked_success = self._tuck_arms(tuck_left=False, tuck_right=False)
        rospy.loginfo(retucked_success)
    	# report success
        
        bin_id = userdata.bin_id
        bin_data = userdata.bin_data.copy()
        bin_data[bin_id] = bin_data[bin_id]._replace(succeeded=True)
        userdata.output_bin_data = bin_data

        return outcomes.DROP_OFF_ITEM_SUCCESS

