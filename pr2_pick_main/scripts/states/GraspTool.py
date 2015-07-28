from actionlib import SimpleActionClient
from copy import copy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_pick_main import handle_service_exceptions
import rospy
from shape_msgs.msg import SolidPrimitive
import smach
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

import outcomes
import visualization as viz


class GraspOrReleaseTool(smach.State):
    '''
    Parent class for grasping and putting back the tool.
    '''
    arm_side = 'l'
    tool_name = 'tool'

    # approximate tool dimensions
    tool_x_size = 0.26
    tool_y_size = 0.01
    tool_z_size = 0.03

    # tool position relative to wrist_roll_link
    tool_x_pos = 0.29
    tool_y_pos = 0.0
    tool_z_pos = 0.0

    waypoint_duration = rospy.Duration(2.0)

    joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upper_arm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    ]

    # hard coded position for grabbing the tool, assumes start position is
    # the result of untucking the arm
    pre_grasp_waypoints = [
        [
            0.15878051573112972,    # shoulder_pan_joint,
            -0.09034277131126116,   # shoulder_lift_joint,
            -0.03542525533930241,   # upper_arm_roll_joint,
            -2.1218775235866243,    # elbow_flex_joint,
            -0.16472694535381027,   # forearm_roll_joint,
            -0.9950645897851714,    # wrist_flex_joint,
            1.6438140418623277      # wrist_roll_joint,
        ]
    ]
    post_grasp_waypoints = [
        [
            0.15878051573112972,    # shoulder_pan_joint,
            -0.09034277131126116,   # shoulder_lift_joint,
            -0.03542525533930241,   # upper_arm_roll_joint,
            -2.1218775235866243,    # elbow_flex_joint,
            -0.16472694535381027,   # forearm_roll_joint,
            -0.221745315012964,     # wrist_flex_joint,
            1.6438140418623277      # wrist_roll_joint,
        ],
        [
            0.22178954492243896,  # shoulder_pan_joint,
            0.08916765551639168,  # shoulder_lift_joint,
            0.1212414100240764,   # upper_arm_roll_joint,
            -1.9155792598650974,  # elbow_flex_joint,
            -1.825748200052204,   # forearm_roll_joint,
            -1.756862271808974,   # wrist_flex_joint,
            1.6335148589202468    # wrist_roll_joint,
        ],
        [
            0.1478368422400076,   # shoulder_pan_joint,
            0.5075741451910245,   # shoulder_lift_joint,
            0.07666276829324792,  # upper_arm_roll_joint,
            -2.1254967913712126,  # elbow_flex_joint,
            -3.2490637932,        # forearm_roll_joint,
            -1.6188519772530534,  # wrist_flex_joint,
            -0.08595766341572286  # wrist_roll_joint,
        ]
    ]
    def __init__(self, **services):
        rospy.loginfo("\nIn Grasping tool.\n")
        smach.State.__init__(
            self,
            outcomes=[
                self.success_value,
                self.failure_value
            ],
            input_keys=['debug'],
        )
        self._attached_collision_objects = services['attached_collision_objects']
        self._drive_to_pose = services['drive_to_pose']
        self._interactive_markers = services['interactive_marker_server']
        self._markers = services['markers']
        self._set_grippers = services['set_grippers']
        self._tf_listener = services['tf_listener']
        self._tts = services['tts']
        self._tuck_arms = services['tuck_arms']

        # wait for marker server
        #while self._markers.get_num_connections() == 0:
        #    rospy.Rate(1).sleep()

        self.arm = SimpleActionClient(
            '{}_arm_controller/joint_trajectory_action'.format(self.arm_side),
            JointTrajectoryAction,
        )
        rospy.loginfo('Waiting for joint trajectory action server')
        self.arm.wait_for_server()

    def execute_trajectory(self, waypoints, debug):
        goal = JointTrajectoryGoal()
        joint_names = ['{}_{}'.format(self.arm_side, joint) for joint in self.joints]
        goal.trajectory.joint_names = joint_names
        for (idx, waypoint) in enumerate(waypoints):

            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = self.waypoint_duration * (1 + idx)
            goal.trajectory.points.append(point)

        return self.arm.send_goal_and_wait(goal)

    def add_tool_collision_object(self):
        '''
        Add an attached collision object representing the tool to the moveit
        planning scene so moveit can plan knowing that the tool is solid and
        moves with the robot's gripper.
        '''

        box = SolidPrimitive(type=SolidPrimitive.BOX)
        box.dimensions = [self.tool_x_size, self.tool_y_size, self.tool_z_size]
        box_pose = Pose()
        box_pose.position.x = self.tool_x_pos
        box_pose.position.y = self.tool_y_pos
        box_pose.position.z = self.tool_z_pos

        collision_object = CollisionObject()
        collision_object.header.frame_id = '{}_wrist_roll_link'.format(self.arm_side)
        collision_object.id = self.tool_name
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives = [box]
        collision_object.primitive_poses = [box_pose]

        tool = AttachedCollisionObject()
        tool.link_name = '{}_wrist_roll_link'.format(self.arm_side)
        joints_below_forearm = [
            '{}_forearm_link', '{}_wrist_flex_link', '{}_wrist_roll_link',
            '{}_gripper_palm_link', '{}_gripper_l_finger_link',
            '{}_gripper_l_finger_tip_link', '{}_gripper_motor_accelerometer_link',
            '{}_gripper_r_finger_link', '{}_gripper_r_finger_tip_link',
        ]
        tool.touch_links = [
            link.format(self.arm_side)
            for link in joints_below_forearm
        ]
        tool.object = collision_object

        for i in range(10):
            self._attached_collision_objects.publish(tool)

        pose_stamped = PoseStamped(header=collision_object.header, pose=box_pose)

        rospy.loginfo("Publishing tool")
        for i in range(5):

            viz.publish_bounding_box(
                self._interactive_markers, pose_stamped,
                self.tool_x_size, self.tool_y_size, self.tool_z_size,
                0.5, 0.2, 0.1, 0.9,
                2,
            )

    def remove_tool_collision_object(self):
        ''' To remove the collision object once we set down the tool '''

        tool = AttachedCollisionObject()
        tool.link_name = '{}_wrist_roll_link'.format(self.arm_side)
        tool.object.id = self.tool_name
        tool.object.operation = CollisionObject.REMOVE

        for i in range(10):
            self._attached_collision_objects.publish(tool)

        viz.delete_bounding_box(
            self._interactive_markers, viz.IdTable.get_id(self.tool_name)
        )

    def back_up(self, userdata):
        '''
        Move the robot away from the shelf so we don't hit things by waving around
        the tool.
        '''
        rospy.loginfo("Backing up to safely get out tool")
        position, quaternion = self._tf_listener.lookupTransform("shelf", "base_footprint", t)

        # find the target pose in robot coordinates
        target_in_shelf_frame = PoseStamped(
            header=Header(frame_id='shelf'),
            pose=Pose(
                position=Point(x=-1.60, y=position[1], z=0.0),
                orientation=Quaternion(w=1, x=0, y=0, z=0),
            ),
        )

        # Visualize target pose.
        viz.publish_base(self._markers, target_in_shelf_frame)

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        self._drive_to_pose.wait_for_service()
        self._drive_to_pose(pose=target_in_shelf_frame, linearVelocity=0.1, angularVelocity=0.1)


class GraspTool(GraspOrReleaseTool):
    name = 'GRASP_TOOL'
    dropping_off = False
    success_value = outcomes.GRASP_TOOL_SUCCESS
    failure_value = outcomes.GRASP_TOOL_FAILURE

    @handle_service_exceptions(outcomes.GRASP_TOOL_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Grasping tool.")
        #self._tuck_arms(False, False)  # untuck arms
        self._tts.publish('Grasping tool')
        #self._set_grippers(True, True, -1)  # open grippers
        if userdata.debug:
            raw_input('(Debug) Press enter to continue:')

        #self.back_up(userdata)

        self.execute_trajectory(self.pre_grasp_waypoints, userdata.debug)
        #self._set_grippers(False, False, -1)  # close both grippers
        self.add_tool_collision_object()
        self.execute_trajectory(self.post_grasp_waypoints, userdata.debug)
        return self.success_value


class ReleaseTool(GraspOrReleaseTool):
    name = 'RELEASE_TOOL'
    dropping_off = True
    success_value = outcomes.RELEASE_TOOL_SUCCESS
    failure_value = outcomes.RELEASE_TOOL_FAILURE

    @handle_service_exceptions(outcomes.RELEASE_TOOL_FAILURE)
    def execute(self, userdata):
        self._tts.publish('Replacing tool')
        if userdata.debug:
            raw_input('(Debug) Press enter to continue:')
        self.execute_trajectory(reversed(self.post_grasp_waypoints), userdata.debug)
        self.execute_trajectory(reversed(self.pre_grasp_waypoints), userdata.debug)
        self._set_grippers(True, False, -1)  # open left gripper
        self.remove_tool_collision_object()
        self._tuck_arms(False, False)  # untuck arms
        self._set_grippers(False, False, -1)  # close both grippers
        return self.success_value