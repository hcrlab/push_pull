from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from pr2_pick_main import handle_service_exceptions
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import moveit_commander
import outcomes
import rospy
import smach
import tf
import visualization as viz


class DropOffItem(smach.State): 
    """Deposits the item into the order bin.
    """
    name = 'DROP_OFF_ITEM'

    def __init__(self, set_grippers, drive_linear, moveit_move_arm, tuck_arms,
        tts, set_static_tf, tf_listener, markers, drive_to_pose, **kwargs):
        smach.State.__init__(self,
            outcomes=[
                outcomes.DROP_OFF_ITEM_SUCCESS,
                outcomes.DROP_OFF_ITEM_FAILURE
            ],
            input_keys=['bin_id', 'bin_data'],
            output_keys=['output_bin_data']
        )
        self._tts = tts
        self._set_grippers = set_grippers
        self._drive_linear = drive_linear
        self._moveit_move_arm = moveit_move_arm
        self._tuck_arms = tuck_arms
        self._set_static_tf = set_static_tf
        self._markers = markers
        self._drive_to_pose = drive_to_pose
        self._tf_listener = tf_listener

        self._order_bin_found = False

    def get_position(self, base_frame="odom_combined"):
        self._tf_listener.waitForTransform(base_frame,"base_footprint",rospy.Time(0), 
                                           rospy.Duration(1.0))
        (pos,orient) = self._tf_listener.lookupTransform(base_frame,"base_footprint", 
                                                         rospy.Time(0))
        return pos

    def planningscene_create_box(self, position, size, name):
        """Create a box in the MoveIt planning scene specified by position and size, both tuples"""
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object(name)
        table_pose = PoseStamped()
        table_pose.header.frame_id = "odom_combined"
        table_pose.pose.position.x = position[0]
        table_pose.pose.position.y = position[1]
        table_pose.pose.position.z = position[2]
        for i in range(10):
            rospy.sleep(0.10)
            scene.add_box(name, table_pose, size)

    @handle_service_exceptions(outcomes.DROP_OFF_ITEM_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Dropping off item from bin {}'.format(userdata.bin_id))
        self._tts.publish('Dropping off item from bin {}'.format(userdata.bin_id))

        if not self._order_bin_found:
            rospy.loginfo('Creating a static TF for the order bin relative to the shelf')

            # # Publish static transform order_bin_fixedto_odom.
            # self._tf_listener.waitForTransform("odom_combined","shelf",rospy.Time(0), rospy.Duration(1.0))
            # (pos_shelf_in_odom, orient_shelf_in_odom) = \
            #     self._tf_listener.lookupTransform("odom_combined", "shelf", rospy.Time(0))
            # # set pitch and roll to 0
            # euler = tf.transformations.euler_from_quaternion(orient_shelf_in_odom)
            # roll = euler[0]
            # pitch = euler[1]
            # yaw = euler[2]
            # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            # # do the actual message publishing
            # order_bin_fixedto_odom = TransformStamped()
            # order_bin_fixedto_odom.header.frame_id = 'odom_combined'
            # order_bin_fixedto_odom.header.stamp = rospy.Time.now()
            # order_bin_fixedto_odom.transform.translation.x = -36 * 0.0254 + pos_shelf_in_odom[0]
            # order_bin_fixedto_odom.transform.translation.y = -32 * 0.0254 + pos_shelf_in_odom[1]  # -27
            # order_bin_fixedto_odom.transform.translation.z = 12 * 0.0254 + pos_shelf_in_odom[2]
            # order_bin_fixedto_odom.transform.rotation.x = quaternion[0]
            # order_bin_fixedto_odom.transform.rotation.y = quaternion[1]
            # order_bin_fixedto_odom.transform.rotation.z = quaternion[2]
            # order_bin_fixedto_odom.transform.rotation.w = quaternion[3]
            # order_bin_fixedto_odom.child_frame_id = 'order_bin_fixedto_odom'
            # self._set_static_tf.wait_for_service()
            # self._set_static_tf(order_bin_fixedto_odom)

            # # Publish static transform order_bin_fixedto_shelf.
            # self._tf_listener.waitForTransform("shelf","order_bin_fixedto_odom",rospy.Time(0), rospy.Duration(1.0))
            # (pos_orderbin_in_shelf, orient_orderbin_in_shelf) = \
            #     self._tf_listener.lookupTransform("shelf", "order_bin_fixedto_odom", rospy.Time(0))
            # order_bin_fixedto_shelf = TransformStamped()
            # order_bin_fixedto_shelf.header.frame_id = 'shelf'
            # order_bin_fixedto_shelf.header.stamp = rospy.Time.now()
            # order_bin_fixedto_shelf.transform.translation.x = pos_orderbin_in_shelf[0]
            # order_bin_fixedto_shelf.transform.translation.y = pos_orderbin_in_shelf[1]
            # order_bin_fixedto_shelf.transform.translation.z = pos_orderbin_in_shelf[2]
            # order_bin_fixedto_shelf.transform.rotation.x = orient_orderbin_in_shelf[0]
            # order_bin_fixedto_shelf.transform.rotation.y = orient_orderbin_in_shelf[1]
            # order_bin_fixedto_shelf.transform.rotation.z = orient_orderbin_in_shelf[2]
            # order_bin_fixedto_shelf.transform.rotation.w = orient_orderbin_in_shelf[3]
            # order_bin_fixedto_shelf.child_frame_id = 'order_bin_fixedto_shelf'
            # self._set_static_tf.wait_for_service()
            # self._set_static_tf(order_bin_fixedto_shelf)

            # # Finally publish order_bin tf
            # order_bin_tf = TransformStamped()
            # order_bin_tf.header.frame_id = 'order_bin_fixedto_shelf'
            # order_bin_tf.header.stamp = rospy.Time.now()
            # order_bin_tf.transform.rotation.w = 1
            # order_bin_tf.child_frame_id = 'order_bin'
            # self._set_static_tf.wait_for_service()
            # self._set_static_tf(order_bin_tf)

            # Do it the simple way! Assumes the shelf is not too tilted
            # TODO(jstn): move this to FindShelf.
            order_bin_tf = TransformStamped()
            order_bin_tf.header.frame_id = 'shelf'
            order_bin_tf.header.stamp = rospy.Time.now()
            order_bin_tf.transform.translation.x = -36 * 0.0254
            order_bin_tf.transform.translation.y = -27 * 0.0254 # -27
            order_bin_tf.transform.translation.z = 12 * 0.0254
            order_bin_tf.transform.rotation.w = 1
            order_bin_tf.child_frame_id = 'order_bin'
            self._set_static_tf.wait_for_service()
            self._set_static_tf(order_bin_tf)

            self._order_bin_found = True

        # TODO(jstn): move this to FindShelf.
        viz.publish_order_bin(self._markers)

        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

        # # add the order bin to planning scene
        # rospy.loginfo('Adding order bin to planning scene')
        # # near wall of bin only
        # size = (0.9144, 0.01, 0.45)    
        # position = (1.524, -1.10, size[2] / 2.0)
        # # full bin
        # # size = (0.9144, 0.4572, 0.9144)    
        # name = 'order_bin'
        # self.planningscene_create_box(position, size, name)

        # move to the order bin
        rospy.loginfo('Move next to the order bin')

        target_in_order_bin_frame = PoseStamped(
            header=Header(frame_id='order_bin'),
            pose=Pose(
                position=Point(x=-10 * 0.0254,
                               y=28.5 * 0.0254,
                               z=0.0),
                orientation=Quaternion(w=1, x=0, y=0, z=0)
            )
        )

        # Visualize target pose.
        viz.publish_base(self._markers, target_in_order_bin_frame)

        rospy.loginfo('Sending drive command')
        self._drive_to_pose.wait_for_service()
        self._drive_to_pose(pose=target_in_order_bin_frame, linearVelocity=0.1, angularVelocity=0.1)

        # move arm above bin
        rospy.loginfo('Move arm above order bin')
        pose_target = PoseStamped()
        pose_target.header.frame_id = "base_footprint";
        pose_target.pose.position.x = 0.2855
        pose_target.pose.position.y = -0.7551
        # pose_target.pose.position.z = 1.0253  # taller table
        pose_target.pose.position.z = 0.70
        pose_target.pose.orientation.x = 0.6878
        pose_target.pose.orientation.y = -0.6733
        pose_target.pose.orientation.z = -0.1941
        pose_target.pose.orientation.w = -0.1897
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose_target, 0, 0, 0, "right_arm", False)

        # lower arm into bin
        rospy.loginfo('Move arm above order bin')
        pose_target.pose.position.z = 0.45
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose_target, 0.03, 0.1, 0, "right_arm", False)

        # open gripper
        rospy.loginfo('Open gripper')
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(False, True, -1)

        # raise arm
        pose_target.pose.position.z = 0.70
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose_target, 0, 0, 0, "right_arm", False)

        # get back to "untucked" position
        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

    	# report success
        
        bin_id = userdata.bin_id
        bin_data = userdata.bin_data.copy()
        bin_data[bin_id] = bin_data[bin_id]._replace(succeeded=True)
        userdata.output_bin_data = bin_data

        return outcomes.DROP_OFF_ITEM_SUCCESS

