from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import Quaternion, Vector3
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, \
    GetPositionIKRequest, GetPlanningScene
import rospy
from std_msgs.msg import Header
from pr2_pick_manipulation.srv import MoveArmIkRequest
import visualization as viz
import tf
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms, GetGrippers, MoveArmIk
import math
from math import fabs


class Tool(object):

    tool_x_size = 0.12
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
    closest_base_distance_to_shelf = 1.05
    # how far in front of item to position tool tip before attempting to touch object
    pre_application_dist = 0.05

    # how close to the edge of the shelf to pull the tip of the object
    distance_from_edge = 0.05

    ###############
    ## ACTION TYPES
    ###############

    front_center_push = 'front_center_push'
    front_side_push_r = 'front_side_push_r'
    front_side_push_l = 'front_side_push_l'
    side_push_full_contact_r = 'side_push_full_contact_r'
    side_push_full_contact_l = 'side_push_full_contact_l'
    side_push_point_contact_r = 'side_push_point_contact_r'
    side_push_point_contact_l = 'side_push_point_contact_l'
    top_pull = 'top_pull'
    top_sideward_pull_r = 'top_sideward_pull_r'
    top_sideward_pull_l = 'top_sideward_pull_l'

    def __init__(self, bounding_box, action_type, **services):
        #self.debug = userdata.debug
        self.bin_id = 'K'
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
           ends[0]: front corner of first end
           ends[1]: front corner of second end
           ends[2]: rear corner of first end
           ends[3]: rear corner of second end
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
        ends = [
            Point(
                x=point.x * cos_yaw - point.y * sin_yaw + position.x,
                y=point.x * sin_yaw + point.y * cos_yaw + position.y,
                z=point.z + position.z,
            )
            for point in ends_in_item_frame
        ]

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
	    if(success == False):
		success = (move_arm_ik(
                	goal=pose_stamped, arm=MoveArmIkRequest().LEFT_ARM).success and success)
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

    #### ACTION PARAMETERS
    pushing_distance = 0.08
    distance_from_end = 0.02 ## float(raw_input("Distance for application from the end of the object: "))

    def get_application_point(self):
        application_point = Point(0, 0, 0)
        if self.action_type == RepositionAction.front_center_push: 
            application_point.x = self.centroid.x  
            application_point.y = self.centroid.y
            application_point.z = self.centroid.z / 2
        elif self.action_type == RepositionAction.front_side_push_l:
            application_point.x = ((self.centroid.x + self.ends[3].x) / 2.0) + 0.02
            application_point.y = self.ends[3].y - PushAway.distance_from_end 
            application_point.z = self.centroid.z / 2
        elif self.action_type == RepositionAction.front_side_push_r:
            application_point.x = ((self.centroid.x + self.ends[0].x) / 2.0) + 0.02
            application_point.y =  self.ends[0].y + PushAway.distance_from_end 
            application_point.z = self.centroid.z / 2
        return application_point

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''

        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation
        orientation = Quaternion(1.0, 0.0, 0.0, 0.0)

        pre_application_delta.x = -RepositionAction.pre_application_dist

        post_application_delta.x = min(
            (3.0 / 4.0) * self.bounding_box.dimensions.x,
            RepositionAction.bin_depth - self.application_point.x - self.bounding_box.dimensions.y
        )

        # construct pre_application pose, application pose, and final pose
        ## TODO: be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x + pre_application_delta.x - Tool.tool_length - PushAway.pushing_distance/2,
                y=self.cap_y(self.application_point.y + pre_application_delta.y),
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length + PushAway.pushing_distance/2,
                y=self.cap_y(self.application_point.y),
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        post_application_pose = Pose(
            position=Point(
                x=self.application_point.x + post_application_delta.x - Tool.tool_length,
                y=self.cap_y(self.application_point.y + post_application_delta.y),
                z=self.application_point.z + post_application_delta.z,
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

    ### ACTION PARAMETERS
    distance_from_end = 0.02 ## float(raw_input("Distance to push the object: "))
    application_height = 0.09  

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''
        ##side = raw_input("1. Left \n2. Right \n")
        
        if(self.action_type == RepositionAction.side_push_point_contact_l or 
            self.action_type == RepositionAction.side_push_full_contact_l):
            ## Left push
            back_end = self.ends[3]
            front_end = self.ends[1]
            if(self.ends[3].y < self.ends[1].y):
                rospy.loginfo("3 < 1")
                target_end = self.ends[3]
            else:
                target_end = self.ends[1]    

            push_direction_sign = 1
        else:
            ## Right push
            back_end = self.ends[2]
            front_end = self.ends[0]
            if(self.ends[2].y < self.ends[0].y):
                rospy.loginf("2 < 0")
                target_end = self.ends[2]
            else:
                target_end = self.ends[0]  
            push_direction_sign = -1

        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation
        orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
        pre_application_delta.y = 0.02

        distance_x = back_end.x - Tool.tool_length

        if(self.action_type == RepositionAction.side_push_point_contact_r or 
            self.action_type == RepositionAction.side_push_point_contact_l):
            distance_x =  front_end.x + float(raw_input("Distance from front end of object to apply tool: ")) \
                        - Tool.tool_length

        # construct pre_application pose, application pose, and final pose
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=distance_x - 0.10,
                y=self.cap_y(target_end.y  + (pre_application_delta.y * push_direction_sign)),
                z=PushSideways.application_height
            ),
            orientation=orientation,
        )

        application_pose = Pose(
            position=Point(
                x=distance_x,
                y=self.cap_y(target_end.y  + (pre_application_delta.y * push_direction_sign)),
                z=PushSideways.application_height
            ),
            orientation=orientation,
        )

        push_application_pose = Pose(
            position=Point(
                x=distance_x,
                y=target_end.y - (PushSideways.distance_from_end * push_direction_sign),  # halfway between centroid and bin center
                z=PushSideways.application_height
            ),
            orientation=orientation,
        )

        # for movement
        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(push_application_pose, self.frame, False),
            MoveArmStep(pre_application_pose, self.frame, False)
        ]


class PullForward(RepositionAction):
    '''
    Parent class for actions that push away on some point of the object.
    Child classes are parameterized by where on the object they're pushing.
    '''

    ## ACTION PARAMETERS
    # dist to push down on object when pulling forward
    push_down_offset = 0.05 ### Defined as 0.02 somewhere else

    def get_application_point(self):
        application_point = Point(0, 0, 0)
        application_point.x = self.ends[3].x + 0.05
        application_point.y = self.centroid.y
        application_point.z = self.centroid.z + PullForward.push_down_offset + 0.01
        return application_point

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''
        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation

        quaternion = tf.transformations.quaternion_from_euler(math.pi / 2 , 0.0, 0.0)
        #type(pose) = geometry_msgs.msg.Pose
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        pre_application_delta.x = -RepositionAction.pre_application_dist
        pre_application_delta.z = PullForward.push_down_offset 

        post_application_delta.x = - self.centroid.x + (self.bounding_box.dimensions.x / 2.0) \
            + RepositionAction.distance_from_edge


        # construct pre_application pose, application pose, and final pose
        ## TODO: correct poses to prevent hitting the bin wall
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x + pre_application_delta.x - Tool.tool_length,
                y=self.application_point.y,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        above_application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length,
                y=self.application_point.y,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - 0.05,
                y=self.application_point.y,
                z=self.application_point.z ,
            ),
            orientation=orientation,
        )
        pull_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - 0.08,
                y=self.application_point.y,
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        lift_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - 0.08,
                y=self.application_point.y,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        self.trajectory = [
            pre_application_pose, above_application_pose, application_pose, pull_pose, lift_pose, pre_application_pose]
        # Probably want to change this experimentally
        self.collision_checking = [True, True, True, True, True, True]

        self.steps = [
            #MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(above_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pull_pose, self.frame, False),
            MoveArmStep(lift_pose, self.frame, False)
        ]


class TopSideways(RepositionAction):
    '''
    Parent class for actions that push away on some point of the object.
    Child classes are parameterized by where on the object they're pushing.
    '''

    ## ACTION PARAMETERS
    # dist to push down on object when pulling forward
    push_down_offset = 0.055

    def get_application_point(self):
        application_point = Point(0, 0, 0)
        application_point.x = self.centroid.x + 0.05
        application_point.y = self.centroid.y 
        application_point.z = self.centroid.z + TopSideways.push_down_offset
        return application_point

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''
        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation

        quaternion = tf.transformations.quaternion_from_euler(math.pi / 2 , 0.0, 0.0)
        #type(pose) = geometry_msgs.msg.Pose
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        pre_application_delta.x = -RepositionAction.pre_application_dist
        pre_application_delta.z = TopSideways.push_down_offset 

        post_application_delta.x = - self.centroid.x + (self.bounding_box.dimensions.x / 2.0) \
            + RepositionAction.distance_from_edge


        # construct pre_application pose, application pose, and final pose
        ## TODO: correct poses to prevent hitting the bin wall
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x - 0.015 - Tool.tool_length,
                y=self.application_point.y,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        above_application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - 0.01,
                y=self.application_point.y,
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length - 0.01,
                y=self.application_point.y,
                z=self.application_point.z - 0.005,
            ),
            orientation=orientation,
        )
        pre_pull_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length,
                y=self.application_point.y - 0.01,
                z=self.application_point.z - 0.005,
            ),
            orientation=orientation,
        )
        pull_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length,
                y=self.application_point.y - 0.025,
                z=self.application_point.z - 0.005,
            ),
            orientation=orientation,
        )
        lift_pose = Pose(
            position=Point(
                x=self.application_point.x - Tool.tool_length,
                y=self.application_point.y - 0.025,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        self.trajectory = [
            pre_application_pose, above_application_pose, application_pose, pull_pose, lift_pose, pre_application_pose]
        # Probably want to change this experimentally
        self.collision_checking = [True, True, True, True, True, True]

        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(above_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pre_pull_pose, self.frame, False),
            MoveArmStep(pull_pose, self.frame, False),
            MoveArmStep(lift_pose, self.frame, False)
        ]
