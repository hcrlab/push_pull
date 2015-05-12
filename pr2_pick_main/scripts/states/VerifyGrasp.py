from geometry_msgs.msg import PoseStamped
from pr2_pick_main import handle_service_exceptions
from pr2_pick_manipulation.srv import MoveArmRequest
from pr2_pick_perception.srv import CountPointsInBox, CountPointsInBoxRequest
import rospy
import smach
import outcomes
import visualization as viz


class VerifyGrasp(smach.State):
    ''' Grasps an item in the bin. '''
    name = 'VERIFY_GRASP'

    def __init__(self, get_grippers, **kwargs):
        smach.State.__init__(self,
                             outcomes=[outcomes.VERIFY_GRASP_SUCCESS,
                                       outcomes.VERIFY_GRASP_FAILURE,
                                       outcomes.VERIFY_GRASP_RETRY],
                             input_keys=['bin_id', 'debug', 'bin_data',
                                         'current_target', 'item_model'],
                             output_keys=['output_bin_data'])

        self._get_grippers = get_grippers
        self._get_items = kwargs['get_items']
        self._set_items = kwargs['set_items']
        self._move_head = kwargs['move_head']
        self._moveit_move_arm = kwargs['moveit_move_arm']
        self._count_points_in_box = kwargs['count_points_in_box']
        self._markers = kwargs['markers']

    def _check_thin_object(self, debug=False):
        """Holds the item up and checks if it's there.
        """
        request = MoveArmRequest()
        request.goal.header.frame_id = 'torso_lift_link'
        request.goal.pose.position.x = 0.479
        request.goal.pose.position.y = -0.284
        request.goal.pose.position.z = 0.327
        request.goal.pose.orientation.x = 0.015
        request.goal.pose.orientation.y = -0.039
        request.goal.pose.orientation.z = 0.661
        request.goal.pose.orientation.w = 0.749
        request.position_tolerance = 0.01
        request.orientation_tolerance = 0.01
        request.planning_time = 8
        request.group = 'right_arm'
        request.plan_only = False
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(request)
        self._move_head.wait_for_service()
        self._move_head(0, 0, 0, 'r_wrist_roll_link')

        box_request = CountPointsInBoxRequest()
        box_request.frame_id = 'torso_lift_link'
        box_request.center.x = 0.51
        box_request.center.y = -0.284 + 0.225
        box_request.center.z = 0.327 + 0.05 - 0.2 / 2
        box_request.dimensions.x = 0.12
        box_request.dimensions.y = 0.12
        box_request.dimensions.z = 0.2
        response = self._count_points_in_box(box_request)

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'torso_lift_link'
        box_pose.pose.position = box_request.center
        box_pose.pose.orientation.w = 1

        rospy.loginfo('Number of points near hand: {}'.format(response.num_points))
        if debug:
            viz.publish_bounding_box(self._markers, box_pose, 0.12, 0.12, 0.2, 0.5,
                                     0.5, 0.5, 0.25, 2345)
            raw_input('[VerifyGrasp]: Verifying thin grasp. ')
        return response.num_points > 1350

    @handle_service_exceptions(outcomes.VERIFY_GRASP_FAILURE)
    def execute(self, userdata):
        # update bin_data
        output_bin_data = userdata.bin_data.copy()
        bin_id = userdata.bin_id
        output_bin_data[bin_id] = output_bin_data[bin_id]._replace(
            attempts_remaining=output_bin_data[bin_id].attempts_remaining - 1)
        userdata.output_bin_data = output_bin_data

        # check if grasp succeeded
        grasp_succeeded = False
        thin_object = userdata.item_model.zero_width_grasp
        gripper_states = self._get_grippers()
        grasp_succeeded = gripper_states.right_open

        if thin_object:
            grasp_succeeded = self._check_thin_object(debug=userdata.debug)

        # decide what state to go to next
        if grasp_succeeded:
            # go to DropOffItem
            self._get_items.wait_for_service()
            self._set_items.wait_for_service()
            items = self._get_items(bin_id).items
            items.remove(userdata.current_target)
            self._set_items(items, bin_id)
            return outcomes.VERIFY_GRASP_SUCCESS
        else:
            # check if there are attempts remaining
            if output_bin_data[bin_id].attempts_remaining == 0:
                return outcomes.VERIFY_GRASP_FAILURE
            else:
                return outcomes.VERIFY_GRASP_RETRY
