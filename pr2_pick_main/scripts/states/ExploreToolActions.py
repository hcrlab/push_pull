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
from PushPullActions import PushSideways, PullSideways
import time
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from pr2_pick_main.web_interface import WebInterface


class ExploreToolActions(smach.State):

    name = 'EXPLORE_TOOL_ACTIONS'

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[outcomes.TOOL_EXPLORATION_SUCCESS, outcomes.TOOL_EXPLORATION_FAILURE],
            input_keys=['debug', 'is_explore', 'bounding_box', 'current_trial'],
            output_keys=['action_params'])

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
        self._tts = services['tts']
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

        self._interface = WebInterface()
        RepositionAction.compute_param_min_max()

    def log_message(self, message):
        rospy.loginfo(message)
        self._tts.publish(message)
        self._interface.display_message(message)

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
          pre_grasp_pose.pose.position.x = -0.35
          pre_grasp_pose.pose.position.y = 0.0
          pre_grasp_pose.pose.position.z = 0.15
          pre_grasp_pose.pose.orientation.x = 1.0
          pre_grasp_pose.pose.orientation.y = 0.0
          pre_grasp_pose.pose.orientation.z = 0.0
          pre_grasp_pose.pose.orientation.w = 0.0
          success_pre_postition = self._moveit_move_arm(pre_grasp_pose,
                                                  0.005, 0.005, 12, 'left_arm',
                                                  False).success
          return success_pre_postition


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
        self.add_tool_collision_object()
        bounding_box = userdata.bounding_box
        self.add_allowable_collision_box(bounding_box)

        success_pre_position = self.pre_position_tool()
        if not success_pre_position:
            rospy.logwarn("Could not preposition arm for tool use!")

        if userdata.is_explore:
            while(True):
                all_actions = RepositionAction.get_all_actions()
                options = all_actions + ['change object configuration']
                self.log_message('Select action.')
                tool_action = self._interface.ask_choice(
                    'Which action should I try?', options)

                if (tool_action in all_actions):
    
                    RepositionAction.load_params()
                    names, values, mins, maxs = RepositionAction.get_action_params(tool_action)

                    self.log_message('Select parameters.')
                    new_values = self._interface.get_floats(
                        message='Choose parameters for ' + tool_action,
                        param_names=names,
                        param_mins=mins,
                        param_maxs=maxs,
                        param_values=values)

                    RepositionAction.set_action_params(tool_action, names, new_values)
                    RepositionAction.save_params()
                    action = RepositionAction.create_action(tool_action,
                        bounding_box, self.services)
                    self.log_message("Starting action.")
                    success = action.execute()
                    if not success:
                        self.log_message("Could not execute action.")

                    self.pre_position_tool()

                else:
                    self.log_message("Ending trial.")
                    self._tuck_arms.wait_for_service()
                    tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
                    return outcomes.TOOL_EXPLORATION_SUCCESS
        
        else:

            tool_action = userdata.current_trial["action"]
            action = RepositionAction.create_action(tool_action,
                bounding_box, self.services)
            userdata.action_params = action.get_action_param_log()
            self.log_message("Starting action " + tool_action)
            success = action.execute()
            if not success:
                self.log_message("Could not execute action.")
            else:
                self.log_message("Action complete.")
                
            self.pre_position_tool()
            self._tuck_arms.wait_for_service()
            tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
            return outcomes.TOOL_EXPLORATION_SUCCESS
