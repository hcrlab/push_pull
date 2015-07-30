from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetPlanningScene
import rospy
from std_msgs.msg import Header
from pr2_pick_manipulation.srv import MoveArmIkRequest
import visualization as viz
import tf
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetPlanningScene
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms, GetGrippers, MoveArmIk

class RepositionAction(object):
    '''
    Parent class for actions that reposition objects using the tool. We take care
    of visualizing and executing the trajectory. It is up to each child to build
    a trajectory that does the right thing.
    '''
    # distance from tip of tool to wrist roll link
    tool_length = 0.41

    # identifier for publishing points in the visualization
    pose_id = 5340

    center_bin_width = 0.30
    side_bin_width = 0.27
    # keep tool this far away from the bin wall
    bin_wall_tolerance = 0.03

    closest_base_distance_to_shelf = 1.05

    bin_widths = {
        'A': side_bin_width, 'B': center_bin_width, 'C': side_bin_width,
        'D': side_bin_width, 'E': center_bin_width, 'F': side_bin_width,
        'G': side_bin_width, 'H': center_bin_width, 'I': side_bin_width,
        'J': side_bin_width, 'K': center_bin_width, 'L': side_bin_width,
    }

    def __init__(self, bounding_box, application_point, userdata, **services):
        self.debug = userdata.debug
        self.bin_id = 'K'

        self.application_point = application_point
        self.bounding_box = bounding_box
        self.centroid = self.bounding_box.pose.pose.position
        self.frame = self.bounding_box.pose.header.frame_id

        self._markers = services['markers']  # holds the topic to publish markers to
        self._move_arm_ik = services['move_arm_ik']
        self._moveit_move_arm = services['moveit_move_arm']
        self._ik_client = services['ik_client']
        self._tf_listener = tf.TransformListener()

        self.steps = []

        self.trajectory = []

    def cap_y(self, value):
        ''' Cap the given y value so it's safely inside the bin. '''
        width = self.bin_widths[self.bin_id]
        max_y = (width / 2.0) - self.bin_wall_tolerance
        if value > max_y:
            return max_y
        elif value < -max_y:
            return -max_y
        return value

    def max_drivable_x_distance(self):
        tf_listener = self._tf_listener
        (base_position, base_orientation) = tf_listener.lookupTransform(
            'base_footprint', 'shelf', rospy.Time(0))
        rospy.loginfo('base_position {}'.format(base_position))
        return base_position[0] - self.closest_base_distance_to_shelf

    def build_trajectory(self):
        '''
        Fill self.trajectory and with poses representing the end effector's
        path to execute this action. These will be plotted in the visualization.
        Fill self.steps with RepositionSteps for the actual movement.
        '''
        pass

    def visualize_trajectory(self):
        ''' Publish a white 1 cm cube marker at each point in the trajectory. '''
        for (idx, pose) in enumerate(self.trajectory):
            viz.publish_point(
                self._markers,
                self.frame,
                pose.position,
                1.0, 1.0, 1.0, 0.5,
                self.pose_id + idx,
            )

    def execute_trajectory(self):
        '''
        Execute the trajectory represented self.steps. Return True if
        successful, False if not.
        '''
        rospy.loginfo('Repositioning item')

        success = True
        for step in self.steps:
            success = success and step.execute(self.debug)
            if not success:
                break

        return success

    def execute(self):
        self.build_trajectory()
        self.visualize_trajectory()
        return self.execute_trajectory()


class RepositionStep(object):
    ''' Parent class for various steps of RepositionActions. '''
    def execute(self):
        pass


class MoveBaseStep(RepositionStep):
    '''
    Step of a RepositionAction in which the robot moves its base.
    No collision checking.
    '''
    def __init__(self, base_vector, frame):
        self.base_vector = base_vector
        self.frame = frame

    def execute(self, debug, **services):
        success = True

        # construct pose by adding base_vector to current base pose
        tf_listener = services['tf_listener']
        base_pose = tf_listener.transformPose(
            self.frame,
            PoseStamped(header=Header(frame_id='base_footprint')),
        )

        drive_to_pose = services['drive_to_pose']
        target = PoseStamped()
        target.header.frame_id = self.frame
        target.pose.orientation = base_pose.pose.orientation
        target.pose.position = deepcopy(base_pose.pose.position)
        target.pose.position.x += self.base_vector.x
        target.pose.position.y += self.base_vector.y
        target.pose.position.z += self.base_vector.z

        rospy.loginfo('base vector {}'.format(self.base_vector))
        rospy.loginfo('base_pose.pose.position {}'.format(base_pose.pose.position))
        rospy.loginfo('target.pose.position {}'.format(target.pose.position))

        current_in_shelf_frame = tf_listener.transformPose(
            'shelf',
            PoseStamped(header=Header(frame_id='base_footprint')),
        )
        rospy.loginfo('current_in_shelf_frame {}'.format(current_in_shelf_frame))

        target_in_shelf_frame = tf_listener.transformPose('shelf', target)
        rospy.loginfo('target_in_shelf_frame {}'.format(target_in_shelf_frame))

        if debug:
            user_input = raw_input(
                '(Debug) Continue to next base pose or fail PushItem state? [C/f]:')
            if user_input == 'f':
                return False

        drive_to_pose(pose=target, linearVelocity=0.1, angularVelocity=0.1)

        return success


class MoveArmStep(RepositionStep):
    '''
    Step of a RepositionAction in which the robot moves its arm.
    Can parameterize for collision checking (moveit move group) or
    no collision checking (moveit ik).
    '''
    def __init__(self, hand_pose, frame, collision_checking):
        self.hand_pose = hand_pose
        self.frame = frame
        self.collision_checking = collision_checking
        #rospy.logwarn('hand pose {}'.format(hand_pose))

    def execute(self, debug, **services):
        success = True
        ik_client = rospy.ServiceProxy('compute_ik', GetPositionIK)
        moveit_move_arm = rospy.ServiceProxy('moveit_service', MoveArm)
        move_arm_ik = rospy.ServiceProxy('move_arm_ik', MoveArmIk)

        #for (pose, collision_checking) in zip(self.trajectory, self.collision_checking):
        pose_stamped = PoseStamped(
            header=Header(frame_id=self.frame),
            pose=self.hand_pose,
        )
        rospy.loginfo('Next pose: {}'.format(pose_stamped))

        if debug:
            user_input = raw_input(
                '(Debug) Continue to next arm pose or fail PushItem state? [C/f]:')
            if user_input == 'f':
                return False

        rospy.sleep(2.0)
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = 'left_arm'
        ik_request.ik_request.timeout = rospy.Duration(3.0)
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_response = ik_client(ik_request)

        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            success = True
            rospy.loginfo("Ik says it's possible")
        else:
            success = False
            rospy.loginfo("Ik says it's impossible")

        if self.collision_checking:
            rospy.loginfo('Collision checking on, using moveit')
            success = (moveit_move_arm(
                pose_stamped, 0.001, 0.01, 5, 'left_arm', False,) and success)
        else:
            rospy.loginfo('Collision checking off, using IK')
            #success = (move_arm_ik(
            #    goal=pose_stamped, arm=MoveArmIkRequest().LEFT_ARM).success and success)
            success = (moveit_move_arm(
                pose_stamped, 0.001, 0.01, 5, 'left_arm', False,) and success)

        return success