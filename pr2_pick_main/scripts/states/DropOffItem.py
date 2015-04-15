import outcomes
import rospy
import smach
import moveit_commander
import tf
import geometry_msgs.msg

def get_position(base_frame="odom_combined"):
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform(base_frame,"base_footprint",rospy.Time(0), rospy.Duration(1.0))
    (pos,orient) = tf_listener.lookupTransform(base_frame,"base_footprint", rospy.Time(0))
    return pos

def planningscene_create_box(position, size, name):
    """Create a box in the MoveIt planning scene specified by position and size, both tuples"""
    scene = moveit_commander.PlanningSceneInterface()
    scene.remove_world_object(name)
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "odom_combined"
    table_pose.pose.position.x = position[0]
    table_pose.pose.position.y = position[1]
    table_pose.pose.position.z = position[2]
    for i in range(10):
        rospy.sleep(0.10)
        scene.add_box(name, table_pose, size)

class DropOffItem(smach.State):
    """Deposits the item into the order bin.
    """
    name = 'DROP_OFF_ITEM'

    def __init__(self, set_grippers, drive_linear, moveit_move_arm, tuck_arms, tts):
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

    def execute(self, userdata):
        rospy.loginfo('Dropping off item from bin {}'.format(userdata.bin_id))
        self._tts.publish('Dropping off item from bin {}'.format(userdata.bin_id))
        
        rospy.loginfo('Untucking right arm')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(True, False)

        # add the order bin to planning scene
        rospy.loginfo('Adding order bin to planning scene')
        position = (1.524, -1.2005999999999999, 0.4572)    
        size = (0.9144, 0.4572, 0.9144)    
        name = 'order_bin'
        planningscene_create_box(position, size, name)

        # move to the order bin
        rospy.loginfo('Move next to the order bin')
        # target_position = (1.20, -0.5588) # too close
        target_position = (1.1227, -0.41382) # move base, assuming torso is fully down
        position = get_position()
        rospy.loginfo("get_position() = " + str(position))
        displacement = (target_position[0] - position[0], target_position[1] - position[1])
        rospy.loginfo("displacement = " + str(displacement))

        # drive in the x direction
        rospy.loginfo('Drive in the x direction')
        self._drive_linear.wait_for_service()
        self._drive_linear(0.10*(1 if displacement[0] > 0 else -1), 0, abs(displacement[0]))
        # drive in the y direction
        rospy.loginfo('Drive in the y direction')
        self._drive_linear.wait_for_service()
        self._drive_linear(0, 0.10*(1 if displacement[1] > 0 else -1), abs(displacement[1]))

        # move arm
        rospy.loginfo('Move arm above order bin')
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base_footprint";
        pose_target.pose.position.x = 0.2855
        pose_target.pose.position.y = -0.7051
        pose_target.pose.position.z = 1.0253
        pose_target.pose.orientation.x = 0.6878
        pose_target.pose.orientation.y = -0.6733
        pose_target.pose.orientation.z = -0.1941
        pose_target.pose.orientation.w = -0.1897
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose_target, 0, "right_arm")

        # open gripper
        rospy.loginfo('Open gripper')
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(False, True)

    	# report success
        # bin_id = userdata.bin_id
        # bin_data = userdata.bin_data.copy()
        # bin_data[bin_id] = bin_data[bin_id]._replace(succeeded=True)
        # userdata.output_bin_data = bin_data
        return outcomes.DROP_OFF_ITEM_SUCCESS

