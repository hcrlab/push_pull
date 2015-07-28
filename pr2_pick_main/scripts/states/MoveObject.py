from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy
from std_msgs.msg import String
import smach
import tf
from visualization_msgs.msg import Marker
import outcomes
from pr2_pick_main import handle_service_exceptions

class MoveObject(smach.State):
	''' Repositions an item with a tool. '''
	name = 'MOVE_OBJECT'

	def __init__(self, **services):
		smach.State.__init__(
			self,
			outcomes=[
				outcomes.MOVE_OBJECT_SUCCESS,
				outcomes.MOVE_OBJECT_FAILURE,
			],
			input_keys=['debug', 'target_descriptor', 'object_name']
		)

		self._get_planning_scene = services['get_planning_scene']
		self._markers = services['markers']  # holds the topic to publish markers to
		self._move_arm_ik = services['move_arm_ik']
		self._moveit_move_arm = services['moveit_move_arm']
		self._planning_scene_publisher = services['planning_scene_publisher']
		self._tuck_arms = services['tuck_arms']
		self._tf_listener = services['tf_listener']
		self._tts = services['tts']

		self.services = services


	# Pre-move_object position
	def pre_position_tool(self):
		goal = JointTrajectoryGoal()
		goal.trajectory.joint_names = [
			'l_shoulder_pan_joint',
			'l_shoulder_lift_joint',
			'l_upper_arm_roll_joint',
			'l_elbow_flex_joint',
			'l_forearm_roll_joint',
			'l_wrist_flex_joint',
			'l_wrist_roll_joint',
		]
		point = JointTrajectoryPoint()
		point.positions = [
			0.1478368422400076,   # shoulder_pan_joint,
			0.5075741451910245,   # shoulder_lift_joint,
			0.07666276829324792,  # upper_arm_roll_joint,
			-2.1254967913712126,  # elbow_flex_joint,
			-3.2490637932,        # forearm_roll_joint,
			-1.6188519772530534,  # wrist_flex_joint,
			-0.08595766341572286  # wrist_roll_joint,
		]
		point.time_from_start = rospy.Duration(2.0)
		goal.trajectory.points.append(point)

		arm = SimpleActionClient(
			'l_arm_controller/joint_trajectory_action',
			JointTrajectoryAction,
		)
		rospy.loginfo('Waiting for joint trajectory action server')
		arm.wait_for_server()
		return arm.send_goal_and_wait(goal)

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

		rospy.loginfo('Sanity check: yaw should be the only nonzero')
		rospy.loginfo('roll {}, pitch {}, yaw {}'.format(roll, pitch, yaw))

		return yaw


	def get_box_ends(self, item_descriptor):
		'''
		Get the corners of the bounding box from the given descriptor.
		return list of corners
		   ends[0]: front corner of first end
		   ends[1]: front corner of second end
		   ends[2]: rear corner of first end
		   ends[3]: rear corner of second end
		'''
		bounding_box = item_descriptor.planar_bounding_box

		rospy.loginfo('Bounding box dimensions {} {} {}'
					  .format(bounding_box.dimensions.x,
							  bounding_box.dimensions.y,
							  bounding_box.dimensions.z)
					  )

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
		viz.publish_pose(
			self._markers,
			bounding_box.pose,
			0, 0.7, 0, 0.5,
			IdTable.get_id('push_item_object_pose')
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

	@handle_service_exceptions(outcomes.MOVE_OBJECT_FAILURE)
	def execute(self, userdata):	
		self.pre_position_tool()

		rospy.loginfo("Object: " + userdata.object_name)
		ends = self.get_box_ends(userdata.item_descriptor)

		

		return outcomes.MOVE_OBJECT_SUCCESS