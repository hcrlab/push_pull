import outcomes
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
import rospy
import smach
import moveit_commander
import geometry_msgs.msg
from pr2_pick_main import handle_service_exceptions
from pr2_pick_manipulation.srv import GetPose
from pr2_pick_manipulation.srv import MoveArm, MoveArmIkRequest
from std_msgs.msg import Header
import math
import json
import os
import tf

class ExtractItem(smach.State):
    """Extracts the target item from the bin.
    """
    name = 'EXTRACT_ITEM'

    # torso heights by bin row
    top_row_torso_height = 0.37
    second_row_torso_height = 0.28
    third_row_torso_height = 0.16
    bottom_row_torso_height = 0.055

    tall_item_roll = 3.14/4


    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.EXTRACT_ITEM_SUCCESS,
                outcomes.EXTRACT_ITEM_FAILURE
            ],
            input_keys=['bin_id', 'debug', 'target_descriptor', 'item_model', 'bounding_box_pose', 'previous_item'],
            output_keys =['previous_item']
	)

        self._moveit_move_arm = services['moveit_move_arm']
        self._tts = services['tts']
        self._tf_listener = services['tf_listener']
        self._drive_to_pose = services['drive_to_pose']
        self._markers = services['markers']
        self._move_arm_ik = services['move_arm_ik']
        self._move_torso = services['move_torso']
        # Shelf heights
	self._tuck_arms = services['tuck_arms']

        self._shelf_height_a_c = 1.56
        self._shelf_height_d_f = 1.33
        self._shelf_height_g_i = 1.10
        self._shelf_height_j_l = 0.85

        self._shelf_heights = {"A": self._shelf_height_a_c,
                                "B": self._shelf_height_a_c,
                                "C": self._shelf_height_a_c,
                                "D": self._shelf_height_d_f,
                                "E": self._shelf_height_d_f,
                                "F": self._shelf_height_d_f,
                                "G": self._shelf_height_g_i,
                                "H": self._shelf_height_g_i,
                                "I": self._shelf_height_g_i,
                                "J": self._shelf_height_j_l,
                                "K": self._shelf_height_j_l,
                                "L": self._shelf_height_j_l}

        # Grasp Parameters

        self._pre_grasp_dist = 0.35
        self._grasp_height = 0.02
        self._pre_grasp_height = self._grasp_height + 0.01
        self._lift_height = 0.05
        self._extract_dist = 0.35
        self._wait_for_transform_duration = rospy.Duration(5.0)


        self.torso_height_by_bin = \
            {letter: self.top_row_torso_height for letter in ('A', 'B', 'C')}
        self.torso_height_by_bin.update(
            {letter: self.second_row_torso_height for letter in ('D', 'E', 'F')})
        self.torso_height_by_bin.update(
            {letter: self.third_row_torso_height for letter in ('G', 'H', 'I')})
        self.torso_height_by_bin.update(
            {letter: self.bottom_row_torso_height for letter in ('J', 'K', 'L')})

    @handle_service_exceptions(outcomes.EXTRACT_ITEM_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Extracting item in bin {}'.format(userdata.bin_id))
        self._tts.publish('Extracting item in bin {}'.format(userdata.bin_id))

        # Center Arm

        rospy.loginfo('Center Arm')
        pose = geometry_msgs.msg.PoseStamped()
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

        #self._moveit_move_arm.wait_for_service()
        
	#self._moveit_move_arm(pose, 0.01, 0.01, 0, 'left_arm', False)

	self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
	
	success = True
        if success:
            return outcomes.EXTRACT_ITEM_SUCCESS
        else:
            return outcomes.EXTRACT_ITEM_SUCCESS
