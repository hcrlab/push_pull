
from pr2_pick_main import handle_service_exceptions
from pr2_pick_main.web_interface import WebInterface
from pr2_pick_manipulation.srv import TuckArms
from pr2_pick_manipulation.srv import MoveHead
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import Transform, Quaternion, Vector3
import outcomes
import rospy
import smach
import tf
import visualization as viz
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InitializeExploration(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'INITIALIZE_EXPLORATION'

    def __init__(self, **services):

        smach.State.__init__(
            self,
            outcomes=[outcomes.INITIALIZE_SUCCESS, outcomes.INITIALIZE_FAILURE],
            input_keys=['debug'],
            output_keys=['start_pose', 'is_before'])

        self._tts = services['tts']
        self._tuck_arms = services['tuck_arms']
        self._move_head = services['move_head']
        self._tf_listener = services['tf_listener']
        self._markers = services['markers']
        self._drive_to_pose = services['drive_to_pose']
        self._im_server = services['interactive_marker_server']
        self._start_pose = None
        self._move_torso = services['move_torso']
        self._set_static_tf = services['set_static_tf']
        self._set_grippers = services['set_grippers']
        self._get_grippers = services['get_grippers']

        self._interface = WebInterface()

    def log_message(self, message):
        rospy.loginfo(message)
        self._tts.publish(message)
        self._interface.display_message(message)

    @handle_service_exceptions(outcomes.INITIALIZE_FAILURE)
    def execute(self, userdata):

        self._interface.display_message('Starting new session', duration=2)
	
	    # Publish static transform.
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.header.stamp = rospy.Time.now()
        odom = PoseStamped()
        odom.pose.position.x = 0.0
        odom.pose.position.y = 0.0
        odom.pose.position.z = 0.0
        odom.pose.orientation.x = 0.0
        odom.pose.orientation.y = 0.0
        odom.pose.orientation.z = 0.0
        odom.pose.orientation.w = 1.0
        transform.transform.translation = odom.pose.position
        transform.transform.rotation = odom.pose.orientation
        transform.child_frame_id = 'odom_combined'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)

        # Tuck arms
        self.log_message('Setting start pose.')

        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
        tuck_success = True
        if not tuck_success:
            self.log_message('Failed to set start pose')
            return outcomes.INITIALIZE_FAILURE
        else:
            rospy.loginfo('StartPose: TuckArms success')

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
            return outcomes.INITIALIZE_FAILURE

        # Find shelf
        self.log_message('Creating shelf model.')
        rospy.sleep(3)

        shelf_odom = PoseStamped()
        shelf_odom.header.frame_id = 'odom_combined'
        shelf_odom.pose.position.x = 1.14
        shelf_odom.pose.position.y = 0.20
        shelf_odom.pose.position.z = 0.0
        shelf_odom.pose.orientation.x = 0.0
        shelf_odom.pose.orientation.y = 0.0
        shelf_odom.pose.orientation.z = 0.0
        shelf_odom.pose.orientation.w = 1.0

        # Publish static transform for shelf
        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
    
        # Publish static transform for bin
        shelf_depth = 0.87
        bottom_row_z = .806
        center_column_y = 0.0

        bin_translation = Vector3(
            x=-shelf_depth / 2.,
            y=center_column_y,
            z=bottom_row_z)

        transform = TransformStamped(
            header=Header(frame_id='shelf',
                          stamp=rospy.Time.now(), ),
            transform=Transform(translation=bin_translation,
                                rotation=Quaternion(w=1,
                                                    x=0,
                                                    y=0,
                                                    z=0), ),
            child_frame_id='bin_K', )
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)

        # Take the tool
        rospy.loginfo("Waiting for set grippers service")
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(open_left=True, open_right=False, effort =-1)

        ################
        self.log_message('Please hand me the tool.')
        self._interface.ask_choice('Press OK when ready to hand tool.', ['OK'])
        self._interface.display_message('Hand the tool now', duration=3, has_countdown=True)
        self.log_message('Taking tool.')
        ################

        grippers_closed = self._set_grippers(open_left=False, open_right=False, effort=-1)
        gripper_states = self._get_grippers()
        if not gripper_states.left_open:
            rospy.logwarn('Grippers did not close properly, will try closing again.')
            self._set_grippers(open_left=False, open_right=False, effort=-1)
        
        userdata.is_before = True

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')
        return outcomes.INITIALIZE_SUCCESS
