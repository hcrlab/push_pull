import outcomes
import rospy
import smach


class SenseBin(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_BIN'

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=[
                outcomes.SENSE_BIN_SUCCESS,
                outcomes.SENSE_BIN_NO_OBJECTS,
                outcomes.SENSE_BIN_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        return outcomes.SENSE_BIN_SUCCESS
