from pr2_pick_manipulation.srv import MoveTorso, MoveTorsoRequest
from pr2_pick_manipulation.srv import SetGrippers
from pr2_pick_manipulation.srv import TuckArms
from std_msgs.msg import String
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
        self._tts = rospy.Publisher('/festival_tts', String)
        rospy.wait_for_service('tuck_arms_service')
        self._tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
        rospy.wait_for_service('torso_service')
        self._move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
        rospy.wait_for_service('gripper_service')
        self._set_grippers = rospy.ServiceProxy('gripper_service', SetGrippers)

    def execute(self, userdata):
        rospy.loginfo('Setting start pose.')

        tuck_success = self._tuck_arms(True, True)
        if not tuck_success:
            rospy.logerr('StartPose: TuckArms failed')
            self._tts.publish('Failed to tuck arms.')

        torso_success = self._move_torso(MoveTorsoRequest.MIN_HEIGHT)
        if not torso_success:
            rospy.logerr('StartPose: MoveTorso failed')
            self._tts.publish('Failed to set torso.')
        
        grippers_success = self._set_grippers(False, False)
        if not grippers_success:
            rospy.logerr('StartPose: SetGrippers failed')
            self._tts.publish('Failed to close grippers.')
        
        if tuck_success and torso_success and grippers_success:
            return outcomes.START_POSE_SUCCESS
        else:
            self._tts.publish('Start pose failed.')
            return outcomes.START_POSE_FAILURE
