from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
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


class DropOffItem(smach.State): 
    """Deposits the item into the order bin.
    """
    name = 'DROP_OFF_ITEM'

    # The x,y coordinates the base should drive to for dropoffs in the order
    # bin frame
    DROPOFF_POS_BASE_X = -0.2540 
    DROPOFF_POS_BASE_Y = 0.6604
    # The position the arm will move to before it lets go of the object
    DROPOFF_POS_ARM_X = 0.0872
    DROPOFF_POS_ARM_Y = 0.8277
    DROPOFF_POS_ARM_Z = 0.5977
    DROPOFF_QUAT_ARM_X = 0.0008
    DROPOFF_QUAT_ARM_Y = 0.7025
    DROPOFF_QUAT_ARM_Z = 0.0197
    DROPOFF_QUAT_ARM_W = 0.7114
    # The height the arm will start at before lowering into the bin to dropoff
    # object
    DROPOFF_POS_ARM_START_Z = 0.7477

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


        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

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
        self._drive_to_pose(pose=target_in_order_bin_frame, linearVelocity=0.1, angularVelocity=0.1)

        # move arm above bin
        rospy.loginfo('Move arm above order bin')
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
        self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM)

        # lower arm into bin
        rospy.loginfo('Move arm above order bin')
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_Z 
        self._move_arm_ik.wait_for_service()
        self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM)
        # self._moveit_move_arm.wait_for_service()
        # self._moveit_move_arm(pose_target, 0.03, 0.1, 0, "right_arm", False)

        # open gripper
        rospy.loginfo('Open gripper')
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(False, True, -1)

        # raise arm
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_START_Z
        self._move_arm_ik.wait_for_service()
        self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM)
        # self._moveit_move_arm.wait_for_service()
        # self._moveit_move_arm(pose_target, 0, 0, 0, "right_arm", False)

        # get back to "untucked" position
        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

    	# report success
        
        bin_id = userdata.bin_id
        bin_data = userdata.bin_data.copy()
        bin_data[bin_id] = bin_data[bin_id]._replace(succeeded=True)
        userdata.output_bin_data = bin_data

        return outcomes.DROP_OFF_ITEM_SUCCESS

