from geometry_msgs.msg import Point, PointStamped
import math
import rospy
import smach
from std_msgs.msg import Header
import tf

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
    left_column_strafe = 0.53
    center_column_strafe = 0.23
    right_column_strafe = -0.07

    # x-direction displacement of robot base center from shelf base center
    # in shelf coordinates
    robot_distance_from_shelf = -1.1

    # speed in meters per second for driving to each bin
    drive_speed = 0.1

    # how long to wait for transforms to be available
    wait_for_transform_duration = rospy.Duration(5)

    name = 'MOVE_TO_BIN'

    def __init__(self, tts, drive_linear, move_torso):
        '''
        @param drive_linear - service proxy for the drive_linear service
        @param move_torso - service proxy for the move_torso service
        '''
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.MOVE_TO_BIN_SUCCESS,
                outcomes.MOVE_TO_BIN_FAILURE
            ],
            input_keys=['bin_id', 'base_to_shelf_tf']
        )
        self._tts = tts
        self.drive_linear = drive_linear
        self.move_torso = move_torso

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

    def execute(self, userdata):
        rospy.loginfo('Moving to bin {}'.format(userdata.bin_id))
        self._tts.publish('Moving to bin {}'.format(userdata.bin_id))

        # find the target point in robot coordinates
        listener = tf.TransformListener()
        target_in_shelf_frame = PointStamped(
            header=Header(frame_id='shelf'),
            point=Point(x=self.robot_distance_from_shelf,
                        y=self.strafe_by_bin[userdata.bin_id],
                        z=0.0)
        )
        rospy.loginfo('Waiting for tf...')
        try:
            listener.waitForTransform(
                'base_link',
                'shelf',
                rospy.Time(0),
                self.wait_for_transform_duration
            )
        except:
            return outcomes.MOVE_TO_BIN_FAILURE

        target_in_robot_frame = listener.transformPoint('base_link', target_in_shelf_frame)
        rospy.loginfo('target_in_robot_frame.point {}'.format(target_in_robot_frame.point))

        # calculate x and y velocity components to drive diagonally
        x_distance = target_in_robot_frame.point.x
        y_distance = target_in_robot_frame.point.y
        speed = self.drive_speed
        distance = math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))
        x_velocity = speed * (x_distance / distance)
        y_velocity = speed * (y_distance / distance)

        # go there
        rospy.loginfo('Moving a distance of {} with x_velocity {} and y_velocity {}'
                      .format(distance, x_velocity, y_velocity))
        self.drive_linear.wait_for_service()
        result = self.drive_linear(x_velocity, y_velocity, distance)

        if not result:
            return outcomes.MOVE_TO_BIN_FAILURE

        # set torso height for the given shelf
        self.move_torso.wait_for_service()
        result = self.move_torso(self.torso_height_by_bin[userdata.bin_id])

        if not result:
            return outcomes.MOVE_TO_BIN_FAILURE

        return outcomes.MOVE_TO_BIN_SUCCESS
