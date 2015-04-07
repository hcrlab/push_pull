import outcomes
import rospy
import smach


class ReadContestFile(smach.State):
    """Reads the contest file.
    """
    name = 'READ_CONTEST_FILE'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.READ_CONTEST_SUCCESS,
                outcomes.READ_CONTEST_FAILURE
            ]
        )

    def execute(self, userdata):
        rospy.loginfo('Reading contest file.')
        return outcomes.READ_CONTEST_SUCCESS
