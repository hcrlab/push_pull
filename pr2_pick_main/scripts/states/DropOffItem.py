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
    DROPOFF_POS_BASE_X = -0.6040
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
        arm_above_bin_success = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM, rospy.Duration(5))
        rospy.loginfo(arm_above_bin_success)

        # lower arm into bin
        rospy.loginfo('Move arm above order bin')
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_Z 
        self._move_arm_ik.wait_for_service()
        arm_into_bin_success = self._move_arm_ik(goal=pose_target, arm=MoveArmIkRequest.RIGHT_ARM, duration=rospy.Duration(5))
        rospy.loginfo(arm_into_bin_success)

        # open gripper
        rospy.loginfo('Open gripper')
        self._set_grippers.wait_for_service()
        open_gripper_success = self._set_grippers(False, True, -1)
        rospy.loginfo(open_gripper_success)

        # raise arm
        pose_target.pose.position.z = self.DROPOFF_POS_ARM_START_Z
        self._move_arm_ik.wait_for_service()
        arm_out_of_bin_success = self._move_arm_ik(goal=pose_target, arm=MoveArmIkRequest.RIGHT_ARM, duration=rospy.Duration(1))
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

