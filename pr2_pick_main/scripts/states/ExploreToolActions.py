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
from geometry_msgs.msg import Point,PointStamped, Pose, PoseStamped
import visualization as viz
from visualization import IdTable
from std_msgs.msg import Header
from PushAway import PushAway
from PullForward import PullForward
from PushSideways import PushSideways
from TopSideways import TopSideways
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
        self.tool_name = 'tool'
        self.waypoint_duration = rospy.Duration(10.0)
        self.tool_x_size = 0.16 ########## REAL SIZE OF THE TOOL??? (previously 0.26)
        self.tool_y_size = 0.01
        self.tool_z_size = 0.03
        self.joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upper_arm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    	]

    	# tool position relative to wrist_roll_link
        self.tool_x_pos = 0.24 #### CENTER OF THE TOOL? (previously 0.29)
        self.tool_y_pos = 0.0
        self.tool_z_pos = 0.0

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

    def _publish(self, marker):
        """Publishes a marker to the given publisher.

        We need to wait for rviz to subscribe. If there are no subscribers to the
        topic within 5 seconds, we give up.
        """

        rate = rospy.Rate(1)
        for i in range(5):
            if self._markers.get_num_connections() > 0:
                self._markers.publish(marker)
                return
            rate.sleep()
        rospy.logwarn(
            'No subscribers to the marker publisher, did not publish marker.')

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

        viz.publish_bounding_box(
            self._interactive_markers, pose_stamped,
            self.tool_x_size, self.tool_y_size, self.tool_z_size,
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

        # visualize bounding box, pose, and ends
        viz.publish_bounding_box(
            self._markers,
            bounding_box.pose,
            bounding_box.dimensions.x,
            bounding_box.dimensions.y,
            bounding_box.dimensions.z,
            0.7, 0, 0, 0.25,
            IdTable.get_id('push_item_bounding_box')
        )
        for idx, end in enumerate(ends):
            red = 0.0 if idx < 2 else 0.5
            green = 0.0 if idx < 2 else 0.5
            blue = 0.7 if idx < 2 else 0.0
            viz.publish_point(
                self._markers,
                bounding_box.pose.header.frame_id,
                end,
                red, green, blue, 0.5,
                IdTable.get_id('push_item_end_{}'.format(idx))
            )

        return ends

    def execute_trajectory(self, waypoints):
        goal = JointTrajectoryGoal()
        joint_names = ['{}_{}'.format(self.arm_side, joint) for joint in self.joints]
        goal.trajectory.joint_names = joint_names
        for (idx, waypoint) in enumerate(waypoints):

            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = self.waypoint_duration * (1 + idx)
            goal.trajectory.points.append(point)

        return self.arm.send_goal_and_wait(goal)


    @handle_service_exceptions(outcomes.TOOL_EXPLORATION_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Starting Move Object state")
        remove_object = CollisionObject()
        remove_object.header.frame_id = '{}_wrist_roll_link'.format(self.arm_side)
        remove_object.id = self.tool_name
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
        ends = self.get_box_ends(bounding_box)
        centroid = bounding_box.pose.pose.position
        frame = bounding_box.pose.header.frame_id

        # add the object we're pushing to the allowable collision matrix
        self.add_allowable_collision_box(bounding_box)
        # position at which tip of tool makes contact with object, in cluster frame
        self.application_point = Point(0, 0, 0)


        while(True):

            #############
            print("1. Front center push \n2. Front side push")
            print("3. Side push with full surface contact\n4. Side push with point contact")
            print("5. Top pull \n6. Top sideward pull \n")

            tool_action = raw_input("enter the number of the tool action:")
            #############

            # Front center push
            if(tool_action == '1'):

                target_end = ends[0]
                self.application_point.x = centroid.x  
                self.application_point.y = centroid.y 

                self.application_point.z = centroid.z / 2

                application_point = PointStamped(
                    header=Header(frame_id=frame),
                    point=self.application_point,
                )

                action = PushAway(
                    bounding_box,
                    application_point,
                    userdata,
                    'front_center_push',
                    **self.services
                )

                success = action.execute()

            # Front side push
            elif(tool_action == '2'):

            	side = raw_input("1. Left \n2. Right \n")
                distance_from_end = float(raw_input("Distance for application from the end of the object: "))

                if(side == '1'):
                    target_end = ends[3]
                else:
                    distance_from_end = -distance_from_end
                    target_end = ends[0]

                self.application_point.x = ((centroid.x + target_end.x) / 2.0) + 0.02
                self.application_point.y =  target_end.y - distance_from_end 
                self.application_point.z = centroid.z / 2

                application_point = PointStamped(
                    header=Header(frame_id=frame),
                    point=self.application_point,
                )

                action = PushAway(
                    bounding_box,
                    application_point,
                    userdata,
                    'front_side_push',
                    **self.services
                )

                success = action.execute()

    
            # Side push with full surface contact
            elif(tool_action == '3'):

                # position at which tip of tool makes contact with object, in cluster frame
                self.application_point = Point(0, 0, 0)

                # Apply tool between target end and edge of bin
                self.application_point.x = 0
                self.application_point.y = 0 
                self.application_point.z = 0
                action = PushSideways(bounding_box, self.application_point,
                                      userdata, 'push_full_contact', **self.services)

                success = action.execute()

            # Side push with point contact
            elif(tool_action == '4'):
                # position at which tip of tool makes contact with object, in cluster frame
                self.application_point = Point(0, 0, 0)

                # Apply tool between target end and edge of bin
                self.application_point.x = 0
                self.application_point.y = 0 
                self.application_point.z = 0
                
                action = PushSideways(bounding_box, self.application_point,
                                      userdata, 'push_point_contact', **self.services)

                success = action.execute()

            # Top pull
            elif(tool_action == '5'):
                self.push_down_offset = 0.05
                self.application_point.x = ends[3].x + 0.05
                self.application_point.y = centroid.y
                self.application_point.z = centroid.z + self.push_down_offset + 0.01
                action = PullForward(bounding_box, self.application_point,
                                     'top_pull', userdata, **self.services)
            
                success = action.execute()

            # Top sideward pull
            elif(tool_action == '6'):

                self.push_down_offset = 0.055
                self.application_point.x = centroid.x + 0.05
                self.application_point.y = centroid.y 
                self.application_point.z = centroid.z + self.push_down_offset
                action = TopSideways(bounding_box, self.application_point,
                                     'top_sideward_pull', userdata, **self.services)
            
                success = action.execute()

                self.pre_position_tool()

            if(tool_action == '-1'):
                self._tuck_arms.wait_for_service()
                tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
                return outcomes.TOOL_EXPLORATION_FAILURE

        return outcomes.TOOL_EXPLORATION_SUCCESS

