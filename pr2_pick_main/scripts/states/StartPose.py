from pr2_pick_main import handle_service_exceptions
from pr2_pick_manipulation.srv import TuckArms
from pr2_pick_manipulation.srv import MoveHead
from geometry_msgs.msg import PoseStamped
import outcomes
import rospy
import smach
import tf
import visualization as viz

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class StartPose(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'START_POSE'

    def __init__(self, tts, tuck_arms, move_head, tf_listener, **kwargs):
        """Constructor for this state.

        Args:
          tts: A publisher for the TTS node
          tuck_arms: The tuck arms service proxy.
          move_head: The head service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.START_POSE_SUCCESS, outcomes.START_POSE_FAILURE],
            input_keys=['debug'],
            output_keys=['start_pose'])
        self._tts = tts
        self._tuck_arms = tuck_arms
        self._move_head = move_head
        self._tf_listener = tf_listener
        self._markers = kwargs['markers']
        self._drive_to_pose = kwargs['drive_to_pose']
	self._im_server = kwargs['interactive_marker_server']
        # The location the robot started at.
        # When the robot relocalizes, it goes back to this start pose.
        self._start_pose = None
	self._move_torso = kwargs['move_torso']

    def _adjust_start_pose_orientation(self):
        # After driving around enough, odom_combined seems to have a lot of
        # rotation error. Orient ourselves so we're facing the shelf.
        try:
            shelf_in_shelf = PoseStamped()
            shelf_in_shelf.header.frame_id = 'shelf'
            shelf_in_shelf.header.stamp = rospy.Time(0)
            shelf_in_shelf.pose.orientation.w = 1
            shelf_in_odom = self._tf_listener.transformPose('odom_combined',
                                                            shelf_in_shelf)
            self._start_pose.pose.orientation = shelf_in_odom.pose.orientation
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr('No transform between {} and {} in FindShelf'.format(
                'shelf', self._start_pose.header.frame_id))

    def _drive_to_start_pose(self):
        viz.publish_base(self._markers, self._start_pose)
        self._drive_to_pose.wait_for_service()
        self._drive_to_pose(self._start_pose, 0.1, 0.1)

    @handle_service_exceptions(outcomes.START_POSE_FAILURE)
    def execute(self, userdata):
        
        #for i in range(0,10):
        #   viz.publish_gripper(self._im_server, pre_grasp_pose , 'grasp_target')
        rospy.loginfo('Setting start pose.')
        self._tts.publish('Setting start pose.')
        #self._move_head.wait_for_service()
        #move_head_success = self._move_head(1.5, 0, 1.05, 'base_footprint')
        #if not move_head_success:
        #    rospy.logerr('StartPose: MoveHead failed')
        #    self._tts.publish('Failed to move head.')
        #else:
        #    rospy.loginfo('StartPose: MoveHead success')
	#self._move_torso.wait_for_service()
    #    rospy.loginfo("TESTE2")
	#self._move_torso(0.058899518946705996, True)
        #self._tuck_arms.wait_for_service()
        #tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
        rospy.loginfo("TESTE1")
        tuck_success = True
        if not tuck_success:
            rospy.logerr('StartPose: TuckArms failed')
            self._tts.publish('Failed to tuck arms.')
        else:
            rospy.loginfo('StartPose: TuckArms success')

        if self._start_pose is None:
            try:
                here = PoseStamped()
                here.header.frame_id = 'base_footprint'
                here.header.stamp = rospy.Time(0)
                here.pose.position.x = 0
                here.pose.position.y = 0
                here.pose.position.z = 0
                here.pose.orientation.w = 1
                here.pose.orientation.x = 0
                here.pose.orientation.y = 0
                here.pose.orientation.z = 0
                self._start_pose = self._tf_listener.transformPose(
                    'odom_combined', here)
                userdata.start_pose = self._start_pose
            except:
                rospy.logerr('No transform for start pose.')
                return outcomes.START_POSE_FAILURE
        else:
            self._adjust_start_pose_orientation()
            self._drive_to_start_pose()

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        if tuck_success:
            return outcomes.START_POSE_SUCCESS
        else:
            self._tts.publish('Start pose failed.')
            return outcomes.START_POSE_FAILURE
