import math
import rospy
import smach
import tf

import outcomes
from pr2_pick_manipulation.srv import DriveLinear, MoveTorso


class MoveToBin(smach.State):
    ''' Moves the robot to a bin. '''
    name = 'MOVE_TO_BIN'


    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.MOVE_TO_BIN_SUCCESS,
                outcomes.MOVE_TO_BIN_FAILURE
            ],
            input_keys=['bin_id', 'base_to_shelf_tf']
        )

        self.torso_height_by_bin = \
            {letter: 0.33 for letter in ('A', 'B', 'C')}
        self.torso_height_by_bin.update(
            {letter: 0.24 for letter in ('D', 'E', 'F')})
        self.torso_height_by_bin.update(
            {letter: 0.12 for letter in ('G', 'H', 'I')})
        self.torso_height_by_bin.update(
            {letter: 0.015 for letter in ('J', 'K', 'L')})

        self.strafe_by_bin = \
            {letter: 0.3 for letter in ('A', 'D', 'G', 'J')}
        self.strafe_by_bin.update(
            {letter: 0.0 for letter in ('B', 'E', 'H', 'K')})
        self.strafe_by_bin.update(
            {letter: -0.3 for letter in ('C', 'F', 'I', 'L')})

    def execute(self, userdata):
        rospy.loginfo('Moving to bin {}'.format(userdata.bin_id))

        # get transform from base_link to shelf
        (translation, rotation) = userdata.base_to_shelf_tf
        rospy.loginfo('base to shelf translation: {} rotation: {}'
                      .format(translation, rotation))


        # move roughly to shelf
        # center of robot's base should be 90 cm from center of shelf's base
        x_distance = translation[0] - 0.9
        y_distance = translation[1] + self.strafe_by_bin[userdata.bin_id]
        velocity = 0.1  # meters per second - go slow so as not to overshoot
        distance = math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))
        x_velocity = velocity * (x_distance / distance)
        y_velocity = velocity * (y_distance / distance)
        rospy.loginfo('Moving a distance of {} with x_velocity {} and y_velocity {}'
                      .format(distance, x_velocity, y_velocity))
        drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
        result = drive_linear(x_velocity, y_velocity, distance)

        if not result:
            return outcomes.MOVE_TO_BIN_FAILURE

        # set torso height for the given shelf
        move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
        result = move_torso(self.torso_height_by_bin[userdata.bin_id])

        if not result:
            return outcomes.MOVE_TO_BIN_FAILURE

        return outcomes.MOVE_TO_BIN_SUCCESS
