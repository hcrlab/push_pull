from geometry_msgs.msg import Point, PointStamped
import math
import rospy
import smach
from std_msgs.msg import Header
import tf

import outcomes


class MoveToBin(smach.State):
    ''' Moves the robot to a bin. '''
    name = 'MOVE_TO_BIN'


    def __init__(self, drive_linear, move_torso):
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
        self.drive_linear = drive_linear
        self.move_torso = move_torso

        self.torso_height_by_bin = \
            {letter: 0.33 for letter in ('A', 'B', 'C')}
        self.torso_height_by_bin.update(
            {letter: 0.24 for letter in ('D', 'E', 'F')})
        self.torso_height_by_bin.update(
            {letter: 0.12 for letter in ('G', 'H', 'I')})
        self.torso_height_by_bin.update(
            {letter: 0.015 for letter in ('J', 'K', 'L')})

        self.strafe_by_bin = \
            {letter: 0.53 for letter in ('A', 'D', 'G', 'J')}
        self.strafe_by_bin.update(
            {letter: 0.23 for letter in ('B', 'E', 'H', 'K')})
        self.strafe_by_bin.update(
            {letter: -0.07 for letter in ('C', 'F', 'I', 'L')})

    def execute(self, userdata):
        rospy.loginfo('Moving to bin {}'.format(userdata.bin_id))

        # find the target point in robot coordinates
        listener = tf.TransformListener()
        target_in_shelf_frame = PointStamped(
            header=Header(frame_id='shelf'),
            point=Point(x=-1.1,
                        y=self.strafe_by_bin[userdata.bin_id],
                        z=0.0)
        )
        rospy.loginfo('Waiting for tf...')
        listener.waitForTransform(
            'base_link',
            'shelf',
            rospy.Time(0),
            rospy.Duration(5)
        )
        target_in_robot_frame = listener.transformPoint('base_link', target_in_shelf_frame)

        # calculate x and y velocity components to drive diagonally
        x_distance = target_in_robot_frame.point.x
        y_distance = target_in_robot_frame.point.y
        velocity = 0.1  # meters per second - go slow so as not to overshoot
        distance = math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))
        x_velocity = velocity * (x_distance / distance)
        y_velocity = velocity * (y_distance / distance)

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
