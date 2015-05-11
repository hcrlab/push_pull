from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import math
from pr2_pick_main import handle_service_exceptions
import rospy
import smach
from std_msgs.msg import Header
import tf
import visualization as viz

import outcomes


class MoveToBin(smach.State):
    ''' Moves the robot to a bin. '''

    ### Tunable constants ###

    # torso heights by bin row
    top_row_torso_height = 0.33
    second_row_torso_height = 0.24
    third_row_torso_height = 0.12
    bottom_row_torso_height = 0.015

    # y-direction displacement of robot base center from shelf base center
    # by bin column, in shelf coordinates. Tuned for robot's right arm to
    # be able to access bins
    left_column_strafe = 0.51
    center_column_strafe = 0.2147
    right_column_strafe = -0.0806

    # x-direction displacement of robot base center from shelf base center
    # in shelf coordinates
    robot_distance_from_shelf_d_l = -1.11
    # Careful with this parameter. If too close, it bumps into the shelf and
    # drive service never returns.
    robot_distance_from_shelf_a_c = -1

    # speed in meters per second for driving to each bin
    drive_speed = 0.1

    # how long to wait for transforms to be available
    wait_for_transform_duration = rospy.Duration(5)

    name = 'MOVE_TO_BIN'

    def __init__(self, tts, drive_to_pose, move_head, move_torso, markers, **kwargs):
        '''
        @param tts - service proxy for speech
        @param drive_to_pose - service proxy for the drive_to_pose service
        @param move_head - service proxy for moving the head
        @param move_torso - service proxy for the move_torso service
        @param markers - publisher for markers
        '''
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.MOVE_TO_BIN_SUCCESS,
                outcomes.MOVE_TO_BIN_FAILURE
            ],
            input_keys=['bin_id', 'debug']
        )
        self._tts = tts
        self.drive_to_pose = drive_to_pose
        self.move_head = move_head
        self.move_torso = move_torso
        self.markers = markers
        self._tuck_arms = kwargs['tuck_arms']

        self.torso_height_by_bin = \
            {letter: self.top_row_torso_height for letter in ('A', 'B', 'C')}
        self.torso_height_by_bin.update(
            {letter: self.second_row_torso_height for letter in ('D', 'E', 'F')})
        self.torso_height_by_bin.update(
            {letter: self.third_row_torso_height for letter in ('G', 'H', 'I')})
        self.torso_height_by_bin.update(
            {letter: self.bottom_row_torso_height for letter in ('J', 'K', 'L')})

        self.strafe_by_bin = \
            {letter: self.left_column_strafe for letter in ('A', 'D', 'G', 'J')}
        self.strafe_by_bin.update(
            {letter: self.center_column_strafe for letter in ('B', 'E', 'H', 'K')})
        self.strafe_by_bin.update(
            {letter: self.right_column_strafe for letter in ('C', 'F', 'I', 'L')})

    @handle_service_exceptions(outcomes.MOVE_TO_BIN_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Moving to bin {}'.format(userdata.bin_id))
        self._tts.publish('Moving to bin {}'.format(userdata.bin_id))
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, True)

        robot_distance_from_shelf = self.robot_distance_from_shelf_a_c
        if userdata.bin_id > "C":
            robot_distance_from_shelf = self.robot_distance_from_shelf_d_l

        # find the target pose in robot coordinates
        target_in_shelf_frame = PoseStamped(
            header=Header(frame_id='shelf'),
            pose=Pose(
                position=Point(x=robot_distance_from_shelf,
                               y=self.strafe_by_bin[userdata.bin_id],
                               z=0.0),
                orientation=Quaternion(w=1, x=0, y=0, z=0)
            )
        )

        # Visualize target pose.
        viz.publish_base(self.markers, target_in_shelf_frame)

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        self.drive_to_pose.wait_for_service()
        self.drive_to_pose(pose=target_in_shelf_frame, linearVelocity=0.1, angularVelocity=0.1)

        # set torso height for the given shelf
        self.move_torso.wait_for_service()
        result = self.move_torso(self.torso_height_by_bin[userdata.bin_id])

        # face the head towards the bin
        self.move_head.wait_for_service()
        self.move_head(x=0.0, y=0.0, z=0.0, frame='bin_{}'.format(userdata.bin_id))

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        if not result:
            return outcomes.MOVE_TO_BIN_FAILURE

        return outcomes.MOVE_TO_BIN_SUCCESS
