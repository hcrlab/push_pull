import outcomes
from pr2_pick_perception.msg import Box, Cluster2, BoundingBox
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
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String, Int32, Float32
from PushAwayExperiment import PushAwayExperiment
from PullForward import PullForward
from PushSidewaysExperiment import PushSidewaysExperiment
from TopSidewaysExperiment import TopSidewaysExperiment
import time
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
import rospkg
from pr2_pick_contest.msg import Record, Trial, MoveObjectParams 
from sensor_msgs.msg import PointCloud2, Image
from pr2_pick_perception.msg import Box, Cluster2, BoundingBox
from pr2_pick_contest.msg import Record
from pr2_pick_perception.srv import BoxPoints, BoxPointsRequest, PlanarPrincipalComponentsRequest, \
	DeleteStaticTransformRequest, BoxPointsResponse #, Cluster
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class MoveObjectExperiment(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'MOVE_OBJECT'

    def __init__(self, tts, tf_listener, **services):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.MOVE_OBJECT_SUCCESS, outcomes.MOVE_OBJECT_FAILURE],
            input_keys=['debug', 'bounding_box', 'before_record', 'current_trial', 'target_cluster', 'current_trial_num'])

        self.arm_side = 'l'
        self.tool_name = 'tool'
        self.waypoint_duration = rospy.Duration(10.0)
        # approximate tool dimensions
        self.tool_x_size = 0.26
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
        self.tool_x_pos = 0.29
        self.tool_y_pos = 0.0
        self.tool_z_pos = 0.0
        self._markers = services['markers']
        self._tf_listener = tf_listener
        self.services = services
        self._moveit_move_arm = services['moveit_move_arm']
        self._set_grippers = services['set_grippers']
        self._get_grippers = services['get_grippers']
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
        self.convert_pcl = services['convert_pcl_service']


    def call_find_cluster_bounding_box(self, cluster):

		req = FindClusterBoundingBoxRequest()
		req.cluster = cluster
		service_name = "find_cluster_bounding_box"
		rospy.loginfo("waiting for find_cluster_bounding_box service")
		rospy.wait_for_service(service_name)
		rospy.loginfo("service found")
		serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
		try:
			res = serv(req)
		except rospy.ServiceException, e:
			rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)  
			return 0
		if not res.error_code:
			return (res.pose, res.box_dims)
		else:
			return (None, None)


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
          # Hard code pre grasp state
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

    def save_image(self, image):
        self.after_record.image = image

    @handle_service_exceptions(outcomes.MOVE_OBJECT_FAILURE)
    def execute(self, userdata):

        item_name = userdata.current_trial["item_name"]
        position = userdata.current_trial["position"]
        orientation = userdata.current_trial["orientation"]
        move_object_params = MoveObjectParams()

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
        
                
        #self.execute_trajectory(tool_waypoints)
        #self.execute_trajectory(tool_waypoints)
        bounding_box = userdata.bounding_box
        self.pre_position_tool()
        # grippers_open = self._set_grippers(open_left=True, open_right=True, effort =-1)
        # raw_input("Add tool to the robot ")
        # time.sleep(3)
        # rospy.loginfo("Waiting for set grippers service")
        # self._set_grippers.wait_for_service()
        # grippers_open = self._set_grippers(open_left=False, open_right=False, effort=-1)
        
        # gripper_states = self._get_grippers()
        # if not gripper_states.left_open:
        #     self._set_grippers(open_left=False, open_right=False, effort=-1)

        # self.add_tool_collision_object()
        ends = self.get_box_ends(bounding_box)
        #self.pre_position_tool()
        centroid = bounding_box.pose.pose.position
        frame = bounding_box.pose.header.frame_id

        # add the object we're pushing to the allowable collision matrix
        self.add_allowable_collision_box(bounding_box)

        # position at which tip of tool makes contact with object, in cluster frame
        self.application_point = Point(0, 0, 0)
        param_suffix = ""


        
            
        print("1. Front center push \n2. Front side push")
        print("3. Side push with full surface contact\n4. Side push with point contact")
        print("5. Top pull \n6. Top sideward pull \n")
        tool_action = userdata.current_trial["action"]

        # Front center push
        if(tool_action == 0):

            target_end = ends[0]
            self.application_point.x = centroid.x  
            self.application_point.y = centroid.y 

            self.application_point.z = centroid.z / 2

            application_point = PointStamped(
                header=Header(frame_id=frame),
                point=self.application_point,
            )

            action = PushAwayExperiment(
                bounding_box,
                self.application_point,
                userdata,
                'front_center_push',
                **self.services
            )

            success = action.execute()

        # Front side push
        elif (tool_action == 1) or (tool_action == 2):

            # side = raw_input("1. Left \n2. Right \n")

            # distance_from_end = float(raw_input("Distance for application from the end of the object: "))
            distance_from_end = userdata.current_trial["action_params"]["front_side_push"]["distance_from_end"]

            if(tool_action == 1):
                side = "left"
            else:
                side = "right"

            param_suffix = side + "_distance_from_end_" + str(distance_from_end)

            if(tool_action == 1):
                target_end = ends[3]
            else:
                distance_from_end = -distance_from_end
                target_end = ends[0]

            move_object_params.distance_from_end = Float32(distance_from_end)

            self.application_point.x = ((centroid.x + target_end.x) / 2.0) + 0.02
            self.application_point.y =  target_end.y - distance_from_end 

            self.application_point.z = centroid.z / 2

            application_point = PointStamped(
                header=Header(frame_id=frame),
                point=self.application_point,
            )

            action = PushAwayExperiment(
                bounding_box,
                self.application_point,
                userdata,
                'front_side_push',
                **self.services
            )

            success = action.execute()


        # Side push with full surface contact
        elif(tool_action == 3) or (tool_action == 4):

            # position at which tip of tool makes contact with object, in cluster frame
            self.application_point = Point(0, 0, 0)

            # Apply tool between target end and edge of bin
            self.application_point.x = 0
            self.application_point.y = 0 
            self.application_point.z = 0
            action = PushSidewaysExperiment(bounding_box, self.application_point,
                                  userdata, 'push_full_contact', **self.services)

            if tool_action == 3:
                side = 'left'
            else:
                side = 'right'

            distance_to_push = userdata.current_trial["action_params"]["push_full_contact"]["distance_to_push_forward"]
            distance_from_front_to_apply_tool = userdata.current_trial["action_params"]["push_full_contact"]["distance_from_front_to_apply_tool"]
            action.set_params(side, distance_to_push, distance_from_front_to_apply_tool)

            param_suffix = side + "_distance_to_push_" + str(distance_to_push) + "_distance_from_front_to_apply_tool_" + str(distance_from_front_to_apply_tool)
            move_object_params.distance_to_push_forward = Float32(distance_to_push)
            move_object_params.distance_from_front_to_apply_tool =  Float32(distance_from_front_to_apply_tool)

            success = action.execute()

        # Side push with point contact
        elif(tool_action == 5) or (tool_action == 6):
            # position at which tip of tool makes contact with object, in cluster frame
            self.application_point = Point(0, 0, 0)

            # Apply tool between target end and edge of bin
            self.application_point.x = 0
            self.application_point.y = 0 
            self.application_point.z = 0
            
            action = PushSidewaysExperiment(bounding_box, self.application_point,
                                  userdata, 'push_point_contact', **self.services)

            if tool_action == 5:
                side = 'left'
            else:
                side = 'right'

            distance_to_push = userdata.current_trial["action_params"]["push_point_contact"]["distance_to_push_forward"]
            # distance_from_front_to_apply_tool = userdata.current_trial["action_params"]["push_full_contact"]["distance_from_front_to_apply_tool"]
            action.set_params(side, distance_to_push)

            param_suffix = side + "_distance_to_push_" + str(distance_to_push)
            move_object_params.distance_to_push_forward = Float32(distance_to_push)

            success = action.execute()

        # Top pull
        elif(tool_action == 7):
            self.push_down_offset = 0.05
            self.application_point.x = ends[3].x + 0.05
            self.application_point.y = centroid.y
            self.application_point.z = centroid.z + self.push_down_offset + 0.01
            action = PullForward(bounding_box, self.application_point,
                                 'top_pull', userdata, **self.services)
        
            success = action.execute()

        # Top sideward pull
        elif(tool_action == 8) or (tool_action == 9):

            self.push_down_offset = 0.055
            self.application_point.x = centroid.x + 0.05
            self.application_point.y = centroid.y 
            self.application_point.z = centroid.z + self.push_down_offset
            action = TopSidewaysExperiment(bounding_box, self.application_point,
                                 'top_sideways_pull', userdata, **self.services)
            if tool_action == 8:
                side = 'left'
            else:
                side = 'right'

            distance_to_push = userdata.current_trial["action_params"]["top_sideways_pull"]["distance_to_push_forward"]
            move_object_params.distance_to_push_forward = Float32(distance_to_push)
            # distance_from_front_to_apply_tool = userdata.current_trial["action_params"]["push_full_contact"]["distance_from_front_to_apply_tool"]
            action.set_params(side, distance_to_push)

            param_suffix = side + "_distance_to_push_" + str(distance_to_push) + "_distance_from_front_to_apply_tool_" + str(distance_from_front_to_apply_tool)
        
            success = action.execute()

        self.pre_position_tool()

        rospy.sleep(3.0)

        self.after_record = Record()

        trial = Trial()

        # Save rgb image
        rospy.Subscriber("/head_mount_kinect/rgb/image_color", Image, self.save_image)

        # Publish cluster
        points = pc2.read_points(userdata.target_cluster.pointcloud, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        marker_cluster = viz.publish_cluster(self._markers, point_list,
                                'bin_K','bin_K_items', 0)

        # Save marker
        self.after_record.marker_pointcloud = marker_cluster

        # Convert cluster PointCloud2 to PointCloud
        rospy.loginfo("Waiting for convert_pcl service")
        self.convert_pcl.wait_for_service()
        rospy.loginfo("PCL service found")
        self._cluster2 = Cluster2()
        self._cluster2.pointcloud = self.convert_pcl(userdata.target_cluster.pointcloud).pointcloud      

        # Save pointcloud
        self.after_record.pointcloud2 = self._cluster2.pointcloud


        self._cluster = userdata.target_cluster
        self._cluster2.header = userdata.target_cluster.header
        self._cluster2.pointcloud.header = userdata.target_cluster.header
        self._cluster2.id = userdata.target_cluster.id
        self.after_record.pointcloud2 = self._cluster.pointcloud

        # Get the bounding box
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(self._cluster2.pointcloud)
        box_pose.header.frame_id = self._cluster.header.frame_id
        b_box = BoundingBox()
        b_box.pose = box_pose
        b_box.dimensions = box_dims

        # Publish Bounding Box
        marker_bounding_box = viz.publish_bounding_box(self._markers, box_pose, 
                     (box_dims.x), 
                     (box_dims.y), 
                     (box_dims.z),
                     1.0, 0.0, 0.0, 0.5, 1)

        
        # Saving bounding box
        self.after_record.boundingbox = b_box
        self.after_record.marker_boundingbox = marker_bounding_box
        # userdata.bounding_box = bounding_box

        rospack = rospkg.RosPack()
        bag_file_path = str(rospack.get_path('pr2_pick_main')) + '/data/experiments'
        bag_file_name = "/TRIAL_" + str(userdata.current_trial_num) + "_" + str(item_name) + "_position_" + str(position) + "_orientation_" + str(orientation) + "_action_" + str(tool_action) \
            + param_suffix + ".bag" 

        
        move_object_params.item_name = String(userdata.current_trial["item_name"])
        move_object_params.orientation = Int32(userdata.current_trial["orientation"])
        move_object_params.position = Int32(userdata.current_trial["position"])
        move_object_params.action = Int32(userdata.current_trial["action"])
        

        self.bag = rosbag.Bag(bag_file_path + bag_file_name , 'w')
        trial.before = userdata.before_record
        trial.after = self.after_record
        trial.params = move_object_params
        self.bag.write('trial', trial)
        self.bag.close()

        if(tool_action == '-1'):
            self._tuck_arms.wait_for_service()
            tuck_success = self._tuck_arms(tuck_left=False, tuck_right=False)
            return outcomes.MOVE_OBJECT_FAILURE 

        return outcomes.MOVE_OBJECT_SUCCESS


