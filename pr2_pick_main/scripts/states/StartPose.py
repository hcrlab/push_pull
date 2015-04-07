import outcomes
import rospy
import smach


class StartPose(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'START_POSE'

    def __init__(self):
        smach.State.__init__(self,
            outcomes=[
                outcomes.START_POSE_SUCCESS,
                outcomes.START_POSE_FAILURE
            ]
        )

    def execute(self, userdata):
        rospy.loginfo('Setting start pose.')
        return outcomes.START_POSE_SUCCESS
