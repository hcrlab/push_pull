import outcomes
import rospy
import smach
from pr2_pick_main import handle_service_exceptions
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from actionlib import SimpleActionClient
import tf
import math
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
import visualization as viz
from visualization import IdTable
from std_msgs.msg import Header
from PushPullActions import RepositionAction, Tool
from PushPullActions import PushAway, PullForward
from PushPullActions import PushSideways, TopSideways
import time
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents

class ExploreToolActions(smach.State):

    name = 'EXPLORE_TOOL_ACTIONS'

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[outcomes.TOOL_EXPLORATION_SUCCESS, outcomes.TOOL_EXPLORATION_FAILURE],
            input_keys=['debug', 'bounding_box'])

        self.arm_side = 'l'

        self.joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upper_arm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    	]

        self._markers = services['markers']
        self._tf_listener = services['tf_listener']
        self.services = services
        self._moveit_move_arm = services['moveit_move_arm']
        self._interactive_markers = services['interactive_marker_server']
        self.arm = SimpleActionClient(
            '{}_arm_controller/joint_trajectory_action'.format(self.arm_side),
            JointTrajectoryAction,
        )
        self._tuck_arms = services['tuck_arms']

        rospy.loginfo('Waiting for joint trajectory action server')
        self.arm.wait_for_server()
        self._attached_collision_objects = services['attached_collision_objects']
        self._get_planning_scene = services['get_planning_scene']
        self.moveit_object_name = 'push_item_target_bbox'
        self._planning_scene_publisher = services['planning_scene_publisher']


    def add_allowable_collision_box(self, bounding_box):
        ''' Add the argument to moveit's allowable collision matrix. '''

        # add the box to the planning scene
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object(self.moveit_object_name)
        dimensions = bounding_box.dimensions
        for i in range(10):
            scene.add_box(
                self.moveit_object_name, bounding_box.pose,
                (dimensions.x, dimensions.y, dimensions.z)
            )
            rospy.sleep(0.1)

        # get allowed collision matrix (acm)
        request = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        response = self._get_planning_scene(request)

        # add box to acm and publish
        acm = response.scene.allowed_collision_matrix
        if not self.moveit_object_name in acm.default_entry_names:
            acm.default_entry_names += [self.moveit_object_name]
            acm.default_entry_values += [True]
        planning_scene_diff = PlanningScene(
            is_diff=True,
            allowed_collision_matrix=acm
        )
        self._planning_scene_publisher.publish(planning_scene_diff)


    def add_tool_collision_object(self):
        '''
        Add an attached collision object representing the tool to the moveit
        planning scene so moveit can plan knowing that the tool is solid and
        moves with the robot's gripper.
        '''

        box = SolidPrimitive(type=SolidPrimitive.BOX)
        box.dimensions = [Tool.tool_x_size, Tool.tool_y_size, Tool.tool_z_size]
        box_pose = Pose()
        box_pose.position.x = Tool.tool_x_pos
        box_pose.position.y = Tool.tool_y_pos
        box_pose.position.z = Tool.tool_z_pos

        collision_object = CollisionObject()
        collision_object.header.frame_id = '{}_wrist_roll_link'.format(self.arm_side)
        collision_object.id = Tool.tool_name
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

        viz.publish_bounding_box(
            self._interactive_markers, pose_stamped,
            Tool.tool_x_size, Tool.tool_y_size, Tool.tool_z_size,
            0.5, 0.2, 0.1, 0.9,
            2,
        )

    # Pre-move_object position
    def pre_position_tool(self):
          pre_grasp_pose = PoseStamped()
          pre_grasp_pose.header.frame_id = "bin_K"
          pre_grasp_pose.pose.position.x = -0.40
          pre_grasp_pose.pose.position.y = 0.0
          pre_grasp_pose.pose.position.z = 0.23
          pre_grasp_pose.pose.orientation.x = 1.0
          pre_grasp_pose.pose.orientation.y = 0.0
          pre_grasp_pose.pose.orientation.z = 0.0
          pre_grasp_pose.pose.orientation.w = 0.0
          success_pre_grasp = self._moveit_move_arm(pre_grasp_pose, 
                                                  0.005, 0.005, 12, 'left_arm',
                                                  False).success


    @handle_service_exceptions(outcomes.TOOL_EXPLORATION_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Starting Move Object state")
        remove_object = CollisionObject()
        remove_object.header.frame_id = '{}_wrist_roll_link'.format(self.arm_side)
        remove_object.id = Tool.tool_name
        remove_object.operation = CollisionObject.REMOVE
        tool = AttachedCollisionObject()
        tool.object = remove_object
        self._attached_collision_objects.publish(tool)

        tool_waypoints = [
        [
            0.03617848465218309,    # shoulder_pan_joint, 13
            -0.765112898156303,   # shoulder_lift_joint, 12
            -0.5227552929694661,   # upper_arm_roll_joint, 14
            -1.0424689689364501,    # elbow_flex_joint, 10
            -0.2167002552019189,   # forearm_roll_joint, 11
            0.7089518194136488,     # wrist_flex_joint, 9
            -0.15862392791101892      # wrist_roll_joint, 8
        ]
    	]

        bounding_box = userdata.bounding_box
        self.pre_position_tool()
        self.add_tool_collision_object()
        # add the object we're pushing to the allowable collision matrix
        self.add_allowable_collision_box(bounding_box)
        # position at which tip of tool makes contact with object, in cluster frame

        while(True):

            #############

            #tool_action = raw_input("enter the number of the tool action:")
            options = RepositionAction.all_actions
            options.append('change_object')
            tool_action = self._interface.ask_choice('Which action should I try?', options)
            #self._interface.display_message('Hand the tool to the robot now', duration=3, has_countdown=True)
            
            #############

            # Front center push
            if(tool_action == RepositionAction.front_center_push or 
                tool_action == RepositionAction.front_side_push_r or 
                tool_action == RepositionAction.front_side_push_l):

                action = PushAway(bounding_box,
                    tool_action,
                    **self.services)

            # Side push with full surface contact
            elif(tool_action == RepositionAction.side_push_full_contact_r or 
                tool_action == RepositionAction.side_push_full_contact_l or 
                tool_action == RepositionAction.side_push_point_contact_r or 
                tool_action == RepositionAction.side_push_point_contact_l):

                action = PushSideways(bounding_box,
                    tool_action,
                    **self.services)

            # Top pull
            elif(tool_action == RepositionAction.top_pull):

                action = PullForward(bounding_box,
                    tool_action,
                    **self.services)

            # Top sideward pull
            elif(tool_action == RepositionAction.top_sideward_pull_r or 
                tool_action == RepositionAction.top_sideward_pull_l):

                action = TopSideways(bounding_box,
                    tool_action,
                    **self.services)
            
            elif(tool_action == 'change_object'):
                self._tuck_arms.wait_for_service()
                tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
                return outcomes.TOOL_EXPLORATION_FAILURE

            success = action.execute()
            self.pre_position_tool()

        return outcomes.TOOL_EXPLORATION_SUCCESS

