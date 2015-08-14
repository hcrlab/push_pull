from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
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
import time

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
            input_keys=['bin_id', 'bin_data', 'previous_item'],
            output_keys=['output_bin_data', 'previous_item']
        )
        self._tts = kwargs["tts"]
        self._set_grippers = kwargs["set_grippers"]
        self._drive_linear = kwargs["drive_linear"]
        self._moveit_move_arm = kwargs["moveit_move_arm"]
        self._move_arm_ik = kwargs["move_arm_ik"]
        self._tuck_arms = kwargs["tuck_arms"]
        self._markers = kwargs["markers"]
        self._drive_to_pose = kwargs["drive_to_pose"]
        self._tf_listener = kwargs["tf_listener"]


    @handle_service_exceptions(outcomes.DROP_OFF_ITEM_FAILURE)
    def execute(self, userdata):      

        # open gripper
        raw_input("Press enter to release item")
        rospy.loginfo('Open gripper')
        self._set_grippers.wait_for_service()
        time.sleep(5)
        open_gripper_success = self._set_grippers(True, True, -1)
        rospy.loginfo(open_gripper_success)


        # get back to "untucked" position
        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        retucked_success = self._tuck_arms(tuck_left=False, tuck_right=False)
        rospy.loginfo(retucked_success)

        return outcomes.DROP_OFF_ITEM_SUCCESS
