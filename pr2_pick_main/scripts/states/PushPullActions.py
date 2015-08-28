from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import Quaternion, Vector3
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetPlanningScene
import rospy
from std_msgs.msg import Header, Float32
from pr2_pick_manipulation.srv import MoveArmIkRequest
import visualization as viz
import tf
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms, GetGrippers, MoveArmIk
import math
from math import fabs
from pr2_pick_contest.msg import ActionParams 
import json
import rospkg
import copy
from copy import deepcopy

class Tool(object):

    tool_x_size = 0.16
    tool_y_size = 0.01
    tool_z_size = 0.03

    # distance from wrist roll link to the tool grip side end
    tool_start_distance = 0.16

    # distance from wrist roll link to the tool tip
    tool_length = tool_start_distance + tool_x_size

    # tool position relative to wrist_roll_link
    tool_x_pos = tool_start_distance + tool_x_size/2
    tool_y_pos = 0.0
    tool_z_pos = 0.0

    tool_name = 'tool'


class RepositionAction(object):
    '''
    Parent class for actions that reposition objects using the tool. We take care
    of visualizing and executing the trajectory. It is up to each child to build
    a trajectory that does the right thing.
    '''
    pose_id = 5340

    # identifier for publishing points in the visualization
    bin_width = 0.27
    bin_depth = 0.42

    # keep tool this far away from the bin wall
    bin_wall_tolerance = 0.03

    all_action_parameters = None
    all_action_param_mins = None
    all_action_param_maxs = None

    @staticmethod
    def compute_param_min_max():
        if RepositionAction.all_action_parameters is None:
            RepositionAction.load_params()
        RepositionAction.all_action_param_mins = dict()
        RepositionAction.all_action_param_maxs = dict()
        for a in RepositionAction.all_action_parameters.keys():
            action_params = RepositionAction.all_action_parameters[a]
            action_param_mins = dict()
            action_param_maxs = dict()
            for p in action_params.keys():
                default_value = action_params[p]
                action_param_mins[p] = default_value-0.04
                action_param_maxs[p] = default_value+0.04
            RepositionAction.all_action_param_mins[a] = copy.copy(action_param_mins)
            RepositionAction.all_action_param_maxs[a] = copy.copy(action_param_maxs)

    @staticmethod
    def get_all_actions():
        if RepositionAction.all_action_parameters is None:
            RepositionAction.load_params()
        all_keys = RepositionAction.all_action_parameters.keys()
        all_actions = []
        for action_type in all_keys:
            suffix = action_type[-1]
            if (suffix == '_'):
                all_actions = all_actions + [action_type + 'r']
                all_actions = all_actions + [action_type + 'l']
            else:
                all_actions = all_actions + [action_type]
        return all_actions

    @staticmethod
    def save_params():
        rospack = rospkg.RosPack()
        params_file = str(rospack.get_path('pr2_pick_contest')) + '/config/action_params.json' 
        with open(params_file, 'w') as data_file:
            json.dump(RepositionAction.all_action_parameters, data_file)

    @staticmethod
    def load_params():
        rospack = rospkg.RosPack()
        params_file = str(rospack.get_path('pr2_pick_contest')) + '/config/action_params.json' 
        with open(params_file) as data_file:    
            RepositionAction.all_action_parameters = json.load(data_file)

    @staticmethod
    def get_key_for_action(action_type):
        key = str(action_type)
        suffix = action_type[-2:]
        if suffix == '_r' or suffix == '_l':
            key = action_type[0:len(action_type)-1]
        return key

    @staticmethod
    def get_action_params(action_type):
        if RepositionAction.all_action_parameters is None:
            RepositionAction.load_params()
        key = RepositionAction.get_key_for_action(action_type)
        names = RepositionAction.all_action_parameters[key].keys()
        values = RepositionAction.all_action_parameters[key].values()
        mins = RepositionAction.all_action_param_mins[key].values()
        maxs = RepositionAction.all_action_param_maxs[key].values()
        return names, values, mins, maxs

    @staticmethod
    def set_action_params(action_type, param_names, param_values):
        if RepositionAction.all_action_parameters is None:
            RepositionAction.load_params()
        key = RepositionAction.get_key_for_action(action_type)
        params = RepositionAction.all_action_parameters[key]
        for i in range(len(param_names)):
            params[param_names[i]] = param_values[i]

    @staticmethod
    def create_action(action_type, bounding_box, services):

        # Front center push
        if(action_type == 'front_center_push' or 
            action_type == 'front_side_push_r' or 
            action_type == 'front_side_push_l'):

            action = PushAway(bounding_box, action_type, **services)

        # Side push with full surface contact
        elif(action_type == 'side_push_r' or 
            action_type == 'side_push_l' or 
            action_type == 'side_rotate_r' or 
            action_type == 'side_rotate_l'):

            action = PushSideways(bounding_box, action_type, **services)

        # Top pull
        elif(action_type == 'top_pull'):

            action = PullForward(bounding_box, action_type, **services)

        # Top sideward pull
        elif(action_type == 'top_sideward_pull_r' or 
            action_type == 'top_sideward_pull_l'):

            action = PullSideways(bounding_box, action_type, **services)

        else:
            rospy.logerr('Unknwon action type: ' + action_type)
            action = None

        return action

    def get_param(self, param_name):
        key = RepositionAction.get_key_for_action(self.action_type)
        action_params = RepositionAction.all_action_parameters[key]
        if param_name in action_params.keys():
            return action_params[param_name]
        else:
            rospy.logwarn('Parameter ' + param_name +
                ' not valid for action ' + self.action_type)
            return None

    def get_action_param_log(self):
        param_log = ActionParams()
        param_log.pushing_distance = Float32(self.get_param('pushing_distance'))
        param_log.pre_application_distance = Float32(self.get_param('pre_application_distance'))
        param_log.distance_from_side = Float32(self.get_param('distance_from_side'))
        param_log.percent_distance_from_side = Float32(self.get_param('percent_distance_from_side'))
        param_log.percent_height_from_center = Float32(self.get_param('percent_height_from_center'))
        param_log.distance_from_back = Float32(self.get_param('distance_from_back'))
        param_log.percent_distance_from_front = Float32(self.get_param('percent_distance_from_front'))
        param_log.pulling_distance = Float32(self.get_param('pulling_distance'))
        param_log.contact_point_depth_offset = Float32(self.get_param('contact_point_depth_offset'))
        param_log.contact_point_down_offset = Float32(self.get_param('contact_point_down_offset'))
        param_log.distance_from_top = Float32(self.get_param('distance_from_top'))
        return param_log

    def __init__(self, bounding_box, action_type, **services):

        self.debug = False
        self.bounding_box = bounding_box
        self.action_type = action_type
        self.centroid = self.bounding_box.pose.pose.position
        self.frame = self.bounding_box.pose.header.frame_id

        self._markers = services['markers']  # holds the topic to publish markers to
        self._move_arm_ik = services['move_arm_ik']
        self._moveit_move_arm = services['moveit_move_arm']
        self._ik_client = services['ik_client']
        self._tf_listener = tf.TransformListener()

        self.steps = []
        self.trajectory = []
        self.ends = self.get_box_ends(self.bounding_box)
        self.application_point = self.get_application_point()

        if RepositionAction.all_action_parameters is None:
            RepositionAction.load_params()

    def get_application_point(self):
        rospy.logwarn('Calling default get_application_point')
        application_point = Point(0, 0, 0)
        application_point.x = self.centroid.x  
        application_point.y = self.centroid.y
        application_point.z = self.centroid.z
        return application_point

    def cap_y(self, value):
        ''' Cap the given y value so it's safely inside the bin. '''
        
        max_y = (RepositionAction.bin_width / 2.0) - self.bin_wall_tolerance

        if value > max_y:
            return max_y
        elif value < -max_y:
            return -max_y
        return value

    def get_yaw(self, bounding_box):
        # get euler angles and normalize
        orientation = bounding_box.pose.pose.orientation
        (yaw, pitch, roll) = tf.transformations.euler_from_quaternion(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        )
        yaw = -yaw  # euler_from_quaternion gives it to us backwards
        # make sure it's in the range (-pi, pi]
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw <= -math.pi:
            yaw += 2 * math.pi

        return yaw

    def get_box_ends(self, bounding_box):
        '''
        Get the corners of the bounding box from the given descriptor.
        return list of corners
           ends[0]: front corner of right end
           ends[1]: front corner of left end
           ends[2]: rear corner of right end
           ends[3]: rear corner of left end
        '''

        # First, figure out whether the y-axis points roughly along or opposite
        # the cluster's y-axis
        yaw = self.get_yaw(bounding_box)

        # get yaw sign
        y_sign = 1.0
        if yaw < 0:
            y_sign = -1.0

        # relative to the bounding box's pose
        ends_in_item_frame = [
            Point(
                x=x_sign * bounding_box.dimensions.x / 2.0,
                y=y_sign * bounding_box.dimensions.y / 2.0,
                z=bounding_box.dimensions.z / 2.0,
            )
            for y_sign in [y_sign, -y_sign]
            for x_sign in [1.0, -1.0]
        ]

        # transform to the cluster's pose
        position = bounding_box.pose.pose.position
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        corners = [
            Point(
                x=point.x * cos_yaw - point.y * sin_yaw + position.x,
                y=point.x * sin_yaw + point.y * cos_yaw + position.y,
                z=point.z + position.z,
            )
            for point in ends_in_item_frame
        ]

        # Reorder the corners so they are always consistent
        xs = [corners[0].x, corners[1].x, corners[2].x, corners[3].x]
        ys = [corners[0].y, corners[1].y, corners[2].y, corners[3].y]
        ys_nomax = list(ys)
        ys_nomin = list(ys)
        miny_index = ys.index(min(ys))
        maxy_index = ys.index(max(ys))
        ys_nomin.remove(min(ys))
        ys_nomax.remove(max(ys))
        miny_index2 = ys.index(min(ys_nomin))
        maxy_index2 = ys.index(max(ys_nomax))

        ## Now, maxy_index, maxy_index2 are on the right
        ## and miny_index, miny_index2 are on the left
        ends = [None, None, None, None]

        if (xs[maxy_index] < xs[maxy_index2]):
            ends[1] = corners[maxy_index]
            ends[3] = corners[maxy_index2]
        else:
            ends[3] = corners[maxy_index]
            ends[1] = corners[maxy_index2]

        if (xs[miny_index] < xs[miny_index2]):
            ends[0] = corners[miny_index]
            ends[2] = corners[miny_index2]
        else:
            ends[2] = corners[miny_index]
            ends[0] = corners[miny_index2]

        for i in range(4):
            viz.publish_point(
                self._markers,
                self.frame,
                ends[i],
                0.0, 1.0, 0.5, 0.9,
                self.pose_id + i,
                str(i)
            )

        return ends

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


class MoveArmStep(RepositionStep):
    '''
    Step of a RepositionAction in which the robot moves its arm.
    Can parameterize for collision checking (moveit move group) or
    no collision checking (moveit ik).
    '''
    def __init__(self, hand_pose, frame, collision_checking, speed=1.0):
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

        success = (moveit_move_arm(
            pose_stamped, 0.001, 0.01, 5, 'left_arm', False, 0.75) and success)

        if not self.collision_checking:

            if(success == False):
            	rospy.loginfo('Collision checking off, using IK')
                success = (move_arm_ik(
                    	goal=pose_stamped, arm=MoveArmIkRequest().LEFT_ARM, duration=6.0).success and success)

        return success


class PushAway(RepositionAction):
    '''
    Pushes toward the rear of the shelf, starting at the given application point.

    Possible applications:
    1. When target object is rotated >45 degrees from a graspable position,
    push on a point between centroid and the far end, to rotate the object.
    2. Push a non target object to the back of the bin to create space around
    target object so we can grasp it.
    '''

    def get_application_point(self):
        application_point = Point(0, 0, 0)
        front_length = self.ends[0].y - self.ends[1].y
        object_height = self.bounding_box.dimensions.z
        if self.action_type == "front_center_push":
            application_point.x = ((self.ends[0].x + self.ends[1].x) * 0.5)
            application_point.y = self.centroid.y
            application_point.z = self.centroid.z + self.get_param('percent_height_from_center')*object_height
        elif self.action_type == "front_side_push_l":
            percent = self.get_param('percent_distance_from_side')
            application_point.x = (percent*self.ends[0].x + (1-percent)*self.ends[1].x)
            application_point.y = self.ends[1].y + self.get_param('percent_distance_from_side')*front_length
            application_point.z = self.centroid.z + self.get_param('percent_height_from_center')*object_height
        elif self.action_type == "front_side_push_r":
            percent = self.get_param('percent_distance_from_side')
            application_point.x = ((1-percent)*self.ends[0].x + percent*self.ends[1].x)
            application_point.y =  self.ends[0].y - self.get_param('percent_distance_from_side')*front_length
            application_point.z = self.centroid.z + self.get_param('percent_height_from_center')*object_height
        return application_point

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''

        # orientation of gripper when repositioning, relative to current orientation
        orientation = Quaternion(1.0, 0.0, 0.0, 0.0)

        # construct pre_application pose, application pose, and final pose
        ## TODO: be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - self.get_param('pre_application_distance'),
                y=self.cap_y(self.application_point.y),
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length + self.get_param('pushing_distance'),
                y=self.cap_y(self.application_point.y),
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        post_application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - self.get_param('pre_application_distance'),
                y=self.cap_y(self.application_point.y),
                z=self.application_point.z,
            ),
            orientation=orientation,
        )

        # for visualization
        self.trajectory = [
            pre_application_pose, application_pose, post_application_pose]

        # for actually moving
        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pre_application_pose, self.frame, False),
        ]


class PushSideways(RepositionAction):
    '''
    Stick the tool straight into the bin, then move it left or right.
    Possible applications:
    1. When target object is rotated <45 degrees from a graspable position,
    push sideways on the near end to rotate the object.
    '''
    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''

        orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
        
        if(self.action_type == "side_rotate_l" or 
            self.action_type == "side_push_l"):
            ## Left push
            front_end = self.ends[1]
            back_end = self.ends[3]
            push_direction_sign = 1
            if front_end.y < back_end.y:
                target_y = front_end.y
            else:
                target_y = back_end.y
        else:
            ## Right push
            front_end = self.ends[0]
            back_end = self.ends[2]
            push_direction_sign = -1
            if front_end.y > back_end.y:
                target_y = front_end.y
            else:
                target_y = back_end.y

        side_length = back_end.x - front_end.x
        if(self.action_type == "side_rotate_r" or 
            self.action_type == "side_rotate_l"):
            target_y = front_end.y
            distance_x = front_end.x - Tool.tool_length + self.get_param('percent_distance_from_front')*side_length
        else:
            distance_x = back_end.x - Tool.tool_length + self.get_param('distance_from_back')

        object_height = self.bounding_box.dimensions.z

        # construct pre_application pose, application pose, and final pose
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        start_pose = Pose(
            position=Point(
                x=distance_x - self.get_param('pre_application_distance'),
                y=self.cap_y(target_y + (self.get_param('distance_from_side') * push_direction_sign)),
                z=self.centroid.z + self.get_param('percent_height_from_center')*object_height
            ),
            orientation=orientation,
        )

        side_pose = Pose(
            position=Point(
                x=distance_x,
                y=self.cap_y(target_y + (self.get_param('distance_from_side') * push_direction_sign)),
                z=self.centroid.z + self.get_param('percent_height_from_center')*object_height
            ),
            orientation=orientation,
        )

        push_pose = Pose(
            position=Point(
                x=distance_x,
                y=target_y - (self.get_param('pushing_distance') * push_direction_sign),
                z=self.centroid.z + self.get_param('percent_height_from_center')*object_height
            ),
            orientation=orientation,
        )

        # for movement
        self.steps = [
            MoveArmStep(start_pose, self.frame, False),
            MoveArmStep(side_pose, self.frame, False),
            MoveArmStep(push_pose, self.frame, False),
            MoveArmStep(side_pose, self.frame, False),
            MoveArmStep(start_pose, self.frame, False)
        ]


class PullForward(RepositionAction):
    '''
    Parent class for actions that push away on some point of the object.
    Child classes are parameterized by where on the object they're pushing.
    '''

    def get_application_point(self):
        application_point = Point(0, 0, 0)
        application_point.x = self.centroid.x + self.get_param('contact_point_depth_offset')
        application_point.y = self.centroid.y
        application_point.z = self.centroid.z + self.bounding_box.dimensions.z / 2.0
        return application_point

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''

        tool_pitch = math.pi / 9
        quaternion = tf.transformations.quaternion_from_euler(math.pi / 2 , tool_pitch, 0.0)
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        self.frame = self.bounding_box.pose.header.frame_id
        x_offset = Tool.tool_length * math.cos(tool_pitch)
	z_offset = Tool.tool_length * math.sin(tool_pitch)
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset - self.get_param('pre_application_distance'),
                y=self.application_point.y,
                z=self.application_point.z + self.get_param('distance_from_top') + z_offset,
            ),
            orientation=orientation,
        )
        above_application_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset,
                y=self.application_point.y,
                z=self.application_point.z + self.get_param('distance_from_top') + z_offset,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset,
                y=self.application_point.y,
                z=self.application_point.z - self.get_param('contact_point_down_offset') + z_offset,
            ),
            orientation=orientation,
        )
        pull_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset - self.get_param('pulling_distance'),
                y=self.application_point.y,
                z=self.application_point.z - self.get_param('contact_point_down_offset') + z_offset,
            ),
            orientation=orientation,
        )
        lift_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset - self.get_param('pulling_distance'),
                y=self.application_point.y,
                z=self.application_point.z + self.get_param('distance_from_top') + z_offset,
            ),
            orientation=orientation,
        )
        self.trajectory = [
            pre_application_pose, 
            above_application_pose,
            application_pose,
            pull_pose,
            lift_pose]

        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(above_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pull_pose, self.frame, False),
            MoveArmStep(lift_pose, self.frame, False)
        ]


class PullSideways(RepositionAction):
    '''
    Parent class for actions that push away on some point of the object.
    Child classes are parameterized by where on the object they're pushing.
    '''

    def get_application_point(self):
        application_point = Point(0, 0, 0)
        application_point.x = self.centroid.x + self.get_param('contact_point_depth_offset')
        application_point.y = self.centroid.y
        application_point.z = self.centroid.z + self.bounding_box.dimensions.z / 2.0
        return application_point

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''
        tool_pitch = math.pi / 9
        quaternion = tf.transformations.quaternion_from_euler(math.pi / 2 , tool_pitch, 0.0)
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        self.frame = self.bounding_box.pose.header.frame_id
        x_offset = Tool.tool_length * math.cos(tool_pitch)
	z_offset = Tool.tool_length * math.sin(tool_pitch)

        if(self.action_type == "top_sideward_pull_l"):
            ## Left pull
            pull_direction_sign = 1
        else:
            ## Right pull
            pull_direction_sign = -1

        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset - self.get_param('pre_application_distance'),
                y=self.application_point.y,
                z=self.application_point.z + self.get_param('distance_from_top') + z_offset,
            ),
            orientation=orientation,
        )
        above_application_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset,
                y=self.application_point.y,
                z=self.application_point.z + self.get_param('distance_from_top') + z_offset,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset,
                y=self.application_point.y,
                z=self.application_point.z - self.get_param('contact_point_down_offset') + z_offset,
            ),
            orientation=orientation,
        )
        pull_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset,
                y=self.application_point.y + (pull_direction_sign * self.get_param('pulling_distance')),
                z=self.application_point.z - self.get_param('contact_point_down_offset') + z_offset,
            ),
            orientation=orientation,
        )
        lift_pose = Pose(
            position=Point(
                x=self.application_point.x - x_offset,
                y=self.application_point.y + (pull_direction_sign * self.get_param('pulling_distance')),
                z=self.application_point.z + self.get_param('distance_from_top') + z_offset,
            ),
            orientation=orientation,
        )
        self.trajectory = [
            pre_application_pose, 
            above_application_pose,
            application_pose,
            pull_pose,
            lift_pose,
            pre_application_pose]

        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(above_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pull_pose, self.frame, False),
            MoveArmStep(lift_pose, self.frame, False),
            MoveArmStep(pre_application_pose, self.frame, False),
        ]
