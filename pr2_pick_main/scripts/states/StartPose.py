from pr2_pick_manipulation.srv import MoveTorso, MoveTorsoRequest
from pr2_pick_manipulation.srv import SetGrippers
from pr2_pick_manipulation.srv import TuckArms
from pr2_pick_manipulation.srv import MoveHead
import outcomes
import rospy
import smach


class StartPose(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'START_POSE'

    def __init__(self, tts, tuck_arms, move_torso, set_grippers, move_head):
        """Constructor for this state.

        Args:
          tts: A publisher for the TTS node
          tuck_arms: The tuck arms service proxy.
          move_torso: The torso service proxy.
          set_grippers: The grippers service proxy.
          move_head: The head service proxy.
        """
        smach.State.__init__(self,
            outcomes=[
                outcomes.START_POSE_SUCCESS,
                outcomes.START_POSE_FAILURE
            ]
        )
        self._tts = tts
        self._tuck_arms = tuck_arms
        self._move_torso = move_torso
        self._set_grippers = set_grippers
        self._move_head = move_head

    def execute(self, userdata):
        rospy.loginfo('Setting start pose.')
        self._tts.publish('Setting start pose.')

        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, True)
        if not tuck_success:
            rospy.logerr('StartPose: TuckArms failed')
            self._tts.publish('Failed to tuck arms.')

        self._move_torso.wait_for_service()
        torso_success = self._move_torso(MoveTorsoRequest.MIN_HEIGHT)
        if not torso_success:
            rospy.logerr('StartPose: MoveTorso failed')
            self._tts.publish('Failed to set torso.')
        
        self._set_grippers.wait_for_service()
        grippers_success = self._set_grippers(False, False)
        if not grippers_success:
            rospy.logerr('StartPose: SetGrippers failed')
            self._tts.publish('Failed to close grippers.')

        self._move_head.wait_for_service()
        move_head_success = self._move_head(1.5, -0.35, 1.1, 'base_footprint')
        if not grippers_success:
            rospy.logerr('StartPose: MoveHead failed')
            self._tts.publish('Failed to move head.')
        
        if (tuck_success and torso_success and grippers_success
                and move_head_success):
            return outcomes.START_POSE_SUCCESS
        else:
            self._tts.publish('Start pose failed.')
            return outcomes.START_POSE_FAILURE
