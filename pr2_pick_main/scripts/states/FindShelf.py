from geometry_msgs.msg import PoseStamped, Transform, TransformStamped, \
    Quaternion, Vector3
from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception.msg import ObjectDetectionRequest, ROI2d
from pr2_pick_perception.srv import LocalizeShelfRequest
from std_msgs.msg import Header
import math
import outcomes
import rospy
import smach
import tf
import visualization as viz
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

class FindShelf(smach.State):
    '''Localizes the shelf.
    '''
    name = 'FIND_SHELF'
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

    def __init__(self, tts, localize_object, set_static_tf, markers,
                 tf_listener, **kwargs):
        '''Constructor for this state.

        Args:
          localize_object: The shelf localization service.
          set_static_tf: The service for setting static tfs.
        '''
        smach.State.__init__(
            self,
            outcomes=[outcomes.FIND_SHELF_SUCCESS, outcomes.FIND_SHELF_FAILURE],
            input_keys=['debug', 'bin_id'])
        self._localize_object = localize_object
        self._set_static_tf = set_static_tf
        self._tf_listener = tf_listener
        self._tts = tts
        self._markers = markers
        self._attached_collision_objects = kwargs['attached_collision_objects']
        self._interactive_markers = kwargs['interactive_marker_server']

    def localize_shelf(self):
        '''Calls the object localization service to get the shelf position.

        If the service fails, or it returns a result outside of acceptable
        bounds, then it will try calling the service again, up to a total of 3
        tries.

        Returns: (success, pose), where success is whether or not we got a
        reasonable pose from the service, and pose is a PoseStamped message
        with the shelf's pose in the odom_combined frame.
        '''
        rospy.sleep(10)

        shelf_odom = PoseStamped()
        shelf_odom.header.frame_id = 'odom_combined'
        shelf_odom.pose.position.x = 1.35
        shelf_odom.pose.position.y = 0.20
        shelf_odom.pose.position.z = 0.0
        shelf_odom.pose.orientation.x = 0.0
        shelf_odom.pose.orientation.y = 0.0
        shelf_odom.pose.orientation.z = 0.0
        shelf_odom.pose.orientation.w = 1.0
        success = True 
        return success, shelf_odom

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

    @handle_service_exceptions(outcomes.FIND_SHELF_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Finding shelf.')
        self._tts.publish('Finding shelf.')

        success, shelf_odom = self.localize_shelf()
        
	
	if not success:
            rospy.logerr('[FindShelf]: Failed to localize shelf.')
            return outcomes.FIND_SHELF_FAILURE

        # Project onto the floor.
        shelf_odom.pose.position.z = 0

        # TODO(jstn): Hack! Hack! Hack! ###############################
        #shelf_odom.pose.position.x += 0.03
        #if (userdata.bin_id is None or userdata.bin_id == 'J' or userdata.bin_id
        #    == 'G' or userdata.bin_id == 'D' or userdata.bin_id == 'A'):
        #    shelf_odom.pose.position.x += 0.03
        ###############################################################

        self._tts.publish('Found shelf.')

        # Publish static transform.
        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self._set_static_tf.wait_for_service()
	
        self._set_static_tf(transform)
	

        # Publish marker
        viz.publish_shelf(self._markers, shelf_odom)
	

        # Set up static a transform for each bin relative to shelf.
        # Bin origin is the front center of the bin opening, equidistant
        # from top edge and bottom edge of bin.
        shelf_depth = 0.87

        top_row_z = 1.524
        second_row_z = 1.308
        third_row_z = 1.073
        bottom_row_z = .806

        left_column_y = .2921
        center_column_y = 0.0
        right_column_y = -.2921

        bin_translations = {
            'A': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=top_row_z),
            'B': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=top_row_z),
            'C': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=top_row_z),
            'D': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=second_row_z),
            'E': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=second_row_z),
            'F': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=second_row_z),
            'G': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=third_row_z),
            'H': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=third_row_z),
            'I': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=third_row_z),
            'J': Vector3(x=-shelf_depth / 2.,
                         y=left_column_y,
                         z=bottom_row_z),
            'K': Vector3(x=-shelf_depth / 2.,
                         y=center_column_y,
                         z=bottom_row_z),
            'L': Vector3(x=-shelf_depth / 2.,
                         y=right_column_y,
                         z=bottom_row_z),
        }

        for (bin_id, translation) in bin_translations.items():
	   
            transform = TransformStamped(
                header=Header(frame_id='shelf',
                              stamp=rospy.Time.now(), ),
                transform=Transform(translation=translation,
                                    rotation=Quaternion(w=1,
                                                        x=0,
                                                        y=0,
                                                        z=0), ),
                child_frame_id='bin_{}'.format(bin_id), )
            self._set_static_tf.wait_for_service()
            self._set_static_tf(transform)
        
        self.add_tool_collision_object()
        return outcomes.FIND_SHELF_SUCCESS
