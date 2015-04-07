import outcomes
import rospy
import smach


class PrepareManipulation(smach.State):
    """Sets the robot's pose before manipulating an object or objects in a bin.
    """
    name = 'PREPARE_MANIPULATION'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.PREPARE_MANIPULATION_SUCCESS,
                outcomes.PREPARE_MANIPULATION_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Preparing manipulation for bin {}'.format(userdata.bin_id))
        return outcomes.PREPARE_MANIPULATION_SUCCESS
