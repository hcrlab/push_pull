import outcomes
import rospy
import smach


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
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Moving to bin {}'.format(userdata.bin_id))

        # get transform from base_link to shelf


        ## get transform from base_link to odom
        ## we know the constant transform from odom to shelf
        ## combine those two.




        return outcomes.MOVE_TO_BIN_SUCCESS
