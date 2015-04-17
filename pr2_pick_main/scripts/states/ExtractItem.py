import outcomes
import rospy
import smach
import moveit_commander
import geometry_msgs.msg
from pr2_pick_manipulation.srv import GetPose
from pr2_pick_manipulation.srv import MoveArm
import json
import os
import tf

class ExtractItem(smach.State):
    """Extracts the target item from the bin.
    """
    name = 'EXTRACT_ITEM'

    def __init__(self, tts, moveit_move_arm, tf_listener, **kwargs):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.EXTRACT_ITEM_SUCCESS,
                outcomes.EXTRACT_ITEM_FAILURE
            ],
            input_keys=['bin_id']
        )

        self._moveit_move_arm = moveit_move_arm
        self._tts = tts
        self._tf_listener = tf_listener

        # Shelf heights

        self._shelf_height_a_c = 1.56
        self._shelf_height_d_f = 1.33
        self._shelf_height_g_i = 1.10
        self._shelf_height_j_l = 0.85

        self._shelf_heights = {"A": self._shelf_height_a_c,
                                "B": self._shelf_height_a_c,
                                "C": self._shelf_height_a_c,
                                "D": self._shelf_height_d_f,
                                "E": self._shelf_height_d_f,
                                "F": self._shelf_height_d_f,
                                "G": self._shelf_height_g_i,
                                "H": self._shelf_height_g_i,
                                "I": self._shelf_height_g_i,
                                "J": self._shelf_height_j_l,
                                "K": self._shelf_height_j_l,
                                "L": self._shelf_height_j_l}

        # Grasp Parameters

        self._pre_grasp_dist = 0.35
        self._grasp_height = 0.02
        self._pre_grasp_height = self._grasp_height + 0.01
        self._lift_height = 0.03
        self._extract_dist = 0.3
        self._wait_for_transform_duration = rospy.Duration(5.0)

    def execute(self, userdata):
        rospy.loginfo('Extracting item in bin {}'.format(userdata.bin_id))
        self._tts.publish('Extracting item in bin {}'.format(userdata.bin_id))


        # Get fake object locations

        current_dir = os.path.dirname(__file__)
        relative_path = "../../config/milestone_1_fake_object_locations.json"
        file_path = os.path.join(current_dir, relative_path)

        with open(file_path) as config_file:
            object_data = json.load(config_file)

        item_pose = geometry_msgs.msg.PoseStamped()

        for shelf_bin in object_data["work_order"]:
            if (shelf_bin["bin"] == "bin_" + str(userdata.bin_id) ):
                item_pose.pose.position.x = shelf_bin["pose"]["x"]
                item_pose.pose.position.y = shelf_bin["pose"]["y"]
                item_pose.pose.position.z = shelf_bin["pose"]["z"]
                break
        item_pose.header.frame_id = "shelf"
        self._tf_listener.waitForTransform(
                'base_footprint',
                'shelf',
                rospy.Time(0),
                self._wait_for_transform_duration
            )


        transformed_item_pose = self._tf_listener.transformPose("base_footprint", item_pose)

        shelf_height = self._shelf_heights[userdata.bin_id]

        #self._ee_pose.wait_for_service()
        
        listener.waitForTransform(
                'base_footprint',
                'r_wrist_roll_link',
                rospy.Time(0),
                self._wait_for_transform_duration
            )

        (current_position, current_orientation) = listener.lookupTransform('base_footprint', 'r_wrist_roll_link', rospy.Time(0))#self._ee_pose("right_arm", "r_wrist_roll_link")
        current_pose = geometry_msgs.msg.PoseStamped()
        current_pose.header.frame_id = 'base_footprint'
        current_pose.pose.position.x = current_position[0]
        current_pose.pose.position.y = current_position[1] 
        current_pose.pose.position.z = current_position[2]

        current_pose.pose.orientation.x = current_orientation[0]
        current_pose.pose.orientation.y = current_orientation[1]
        current_pose.pose.orientation.z = current_orientation[2]
        current_pose.pose.orientation.w = current_orientation[3]

        rospy.loginfo("pose x: " + str(current_pose.pose.position.x) + ", y: " + str(current_pose.pose.position.y) + ", z: " + str(current_pose.pose.position.z))
        rospy.loginfo("orientation x: " + str(current_pose.pose.orientation.x) + ", y: " + str(current_pose.pose.orientation.y) + ", z: " + str(current_pose.pose.orientation.z) + ", w: " + str(current_pose.pose.orientation.w))


        success_lift = False
        attempts = 5
        for i in range(attempts):
            if userdata.bin_id > "C":
                # Lift item to clear bin lip
                pose_target = geometry_msgs.msg.PoseStamped()
                pose_target.header.frame_id = 'base_footprint'
                rospy.loginfo("Lift")
                pose_target.pose.position.x = current_pose.pose.position.x
                pose_target.pose.position.y = current_pose.pose.position.y
                pose_target.pose.orientation.x = current_pose.pose.orientation.x
                pose_target.pose.orientation.y = current_pose.pose.orientation.y
                pose_target.pose.orientation.z = current_pose.pose.orientation.z
                pose_target.pose.orientation.w = current_pose.pose.orientation.w 
                pose_target.pose.position.z = current_pose.pose.position.z + self._lift_height
                #pose_target.pose.position.x = current_pose.pose.position.x - 0.01 * i 

                # pose_target.header.frame_id = "base_footprint";
                # pose_target.pose.orientation.w = 1
                # pose_target.pose.position.x = grasp_dist + item_pose.position.x
                # pose_target.pose.position.y = robot_centered_offset #+ item_pose.position.y;
                # pose_target.pose.position.z = shelf_height + grasp_height + item_pose.position.z + lift_height
         
                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                
                self._moveit_move_arm.wait_for_service()
                success_lift = self._moveit_move_arm(pose_target, 0.01 + 0.005 * i, 0.015 + 0.005 * i, 0, "right_arm").success
            else:
                euler_tuple = tf.transformations.euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])
                euler = list(euler_tuple)
                euler[1] = euler[1] + 3.14/6.0
                quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
                # Lift item to clear bin lip
                pose_target = geometry_msgs.msg.PoseStamped()
                pose_target.header.frame_id = 'base_footprint'
                rospy.loginfo("Lift")
                pose_target.pose.position.x = current_pose.pose.position.x
                pose_target.pose.position.y = current_pose.pose.position.y
                pose_target.pose.orientation.x = quaternion[0]
                pose_target.pose.orientation.y = quaternion[1]
                pose_target.pose.orientation.z = quaternion[2]
                pose_target.pose.orientation.w = quaternion[3]
                pose_target.pose.position.z = current_pose.pose.position.z 
                #pose_target.pose.position.x = current_pose.pose.position.x - 0.01 * i 

                # pose_target.header.frame_id = "base_footprint";
                # pose_target.pose.orientation.w = 1
                # pose_target.pose.position.x = grasp_dist + item_pose.position.x
                # pose_target.pose.position.y = robot_centered_offset #+ item_pose.position.y;
                # pose_target.pose.position.z = shelf_height + grasp_height + item_pose.position.z + lift_height
     
                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
            
                self._moveit_move_arm.wait_for_service()
                success_lift = self._moveit_move_arm(pose_target, 0.01 + 0.005 * i, 0.015 + 0.005 * i, 0, "right_arm").success
            if success_lift:
                rospy.loginfo("Lift success")
                break
            else:
                rospy.loginfo("Lift attempt " + str(i) + " failed")
                continue

        attempts = 5

        # Pull item out of bin
        success = False

        for i in range(attempts):

            # Pose in front of bin

            if userdata.bin_id > "C":
                rospy.loginfo("Extract:")
                extract_pose_target = geometry_msgs.msg.PoseStamped()
                extract_pose_target.header.frame_id = "base_footprint";
                extract_pose_target.pose.orientation.w = 1
                extract_pose_target.pose.position.x = self._extract_dist + 0.01 * i 
                extract_pose_target.pose.position.y = transformed_item_pose.pose.position.y
                extract_pose_target.pose.position.z = pose_target.pose.position.z
                

                rospy.loginfo("pose x: " + str(extract_pose_target.pose.position.x) + ", y: " + str(extract_pose_target.pose.position.y) + ", z: " + str(extract_pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(extract_pose_target.pose.orientation.x) + ", y: " + str(extract_pose_target.pose.orientation.y) + ", z: " + str(extract_pose_target.pose.orientation.z) + ", w: " + str(extract_pose_target.pose.orientation.w))
                
                self._moveit_move_arm.wait_for_service()
                success = self._moveit_move_arm(extract_pose_target, 0.01, 0.01, 8.0, "right_arm").success
            else:
                rospy.loginfo("Extract:")
                extract_pose_target = geometry_msgs.msg.PoseStamped()
                extract_pose_target.header.frame_id = "base_footprint";
               
                rospy.loginfo("In top row")
                extract_pose_target.pose.orientation.x = 0.984
                extract_pose_target.pose.orientation.y = -0.013
                extract_pose_target.pose.orientation.z = 0.178
                extract_pose_target.pose.orientation.w = 0.028
                extract_pose_target.pose.position.x = 0.243 + 0.01 * i
                extract_pose_target.pose.position.y = transformed_item_pose.pose.position.y
                extract_pose_target.pose.position.z = 1.508
 

                rospy.loginfo("pose x: " + str(extract_pose_target.pose.position.x) + ", y: " + str(extract_pose_target.pose.position.y) + ", z: " + str(extract_pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(extract_pose_target.pose.orientation.x) + ", y: " + str(extract_pose_target.pose.orientation.y) + ", z: " + str(extract_pose_target.pose.orientation.z) + ", w: " + str(extract_pose_target.pose.orientation.w))
                
                self._moveit_move_arm.wait_for_service()
                success = self._moveit_move_arm(extract_pose_target, 0.01, 0.01, 8.0, "right_arm").success
            if success:
                # Open Hand
                rospy.loginfo("Extract success")
                break
            else:
                rospy.loginfo("Extract attempt " + str(i) + " failed")
                continue

        # Center Arm

        rospy.loginfo('Center Arm')
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = 'base_footprint';

        if userdata.bin_id > 'F':
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.4665;
            pose.pose.position.z = 0.6905;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;
        elif userdata.bin_id > 'C':
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.3865;
            pose.pose.position.z = 0.6905 + 0.23;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;
        else:
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.3865;
            pose.pose.position.z = 0.6905 + 2 * 0.23;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;

        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose, 0.01, 0.01, 0, 'right_arm')


        if success:
            return outcomes.EXTRACT_ITEM_SUCCESS
        else:
            return outcomes.EXTRACT_ITEM_FAILURE
