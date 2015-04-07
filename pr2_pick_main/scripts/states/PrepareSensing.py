import outcomes
import rospy
import smach


class PrepareSensing(smach.State):
    """Sets the robot's pose before sensing a bin.
    """
    name = 'PREPARE_SENSING'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.PREPARE_SENSING_SUCCESS,
                outcomes.PREPARE_SENSING_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Preparing to sense bin {}'.format(userdata.bin_id))
        return outcomes.PREPARE_SENSING_SUCCESS
