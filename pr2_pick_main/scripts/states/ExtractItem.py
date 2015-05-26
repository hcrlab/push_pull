import outcomes
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
import rospy
import smach
import moveit_commander
import geometry_msgs.msg
from pr2_pick_main import handle_service_exceptions
from pr2_pick_manipulation.srv import GetPose
from pr2_pick_manipulation.srv import MoveArm, MoveArmIkRequest
from std_msgs.msg import Header
import json
import os
import tf

class ExtractItem(smach.State):
    """Extracts the target item from the bin.
    """
    name = 'EXTRACT_ITEM'

    # torso heights by bin row
    top_row_torso_height = 0.37
    second_row_torso_height = 0.28
    third_row_torso_height = 0.16
    bottom_row_torso_height = 0.055


    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.EXTRACT_ITEM_SUCCESS,
                outcomes.EXTRACT_ITEM_FAILURE
            ],
            input_keys=['bin_id', 'debug', 'target_descriptor']
        )

        self._moveit_move_arm = services['moveit_move_arm']
        self._tts = services['tts']
        self._tf_listener = services['tf_listener']
        self._drive_to_pose = services['drive_to_pose']
        self._markers = services['markers']
        self._move_arm_ik = services['move_arm_ik']
        self._move_torso = services['move_torso']
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
        self._lift_height = 0.05
        self._extract_dist = 0.35
        self._wait_for_transform_duration = rospy.Duration(5.0)


        self.torso_height_by_bin = \
            {letter: self.top_row_torso_height for letter in ('A', 'B', 'C')}
        self.torso_height_by_bin.update(
            {letter: self.second_row_torso_height for letter in ('D', 'E', 'F')})
        self.torso_height_by_bin.update(
            {letter: self.third_row_torso_height for letter in ('G', 'H', 'I')})
        self.torso_height_by_bin.update(
            {letter: self.bottom_row_torso_height for letter in ('J', 'K', 'L')})

    @handle_service_exceptions(outcomes.EXTRACT_ITEM_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Extracting item in bin {}'.format(userdata.bin_id))
        self._tts.publish('Extracting item in bin {}'.format(userdata.bin_id))


        shelf_height = self._shelf_heights[userdata.bin_id]

        object_pose = self._tf_listener.transformPose('base_footprint', userdata.target_descriptor.planar_bounding_box.pose)
        #self._ee_pose.wait_for_service()
        
        self._tf_listener.waitForTransform(
            'base_footprint',
            'r_wrist_roll_link',
            rospy.Time(0),
            self._wait_for_transform_duration
        )

        (current_position, current_orientation) = self._tf_listener.lookupTransform(
            'base_footprint', 'r_wrist_roll_link', rospy.Time(0))#self._ee_pose("right_arm", "r_wrist_roll_link")
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
       
        attempts = 3
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
                pose_target.pose.position.x = current_pose.pose.position.x - 0.01 * i 

                # pose_target.header.frame_id = "base_footprint";
                # pose_target.pose.orientation.w = 1
                # pose_target.pose.position.x = grasp_dist + item_pose.position.x
                # pose_target.pose.position.y = robot_centered_offset #+ item_pose.position.y;
                # pose_target.pose.position.z = shelf_height + grasp_height + item_pose.position.z + lift_height
         
                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                
                self._move_arm_ik.wait_for_service()
                """
                try:
                    success_lift = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM).success
                except rospy.ServiceException:
                    rospy.sleep(1.0)
                    self._move_arm_ik.wait_for_service()
                    success_lift = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM).success
                """
            else:

                euler_tuple = tf.transformations.euler_from_quaternion(
                                                        [current_pose.pose.orientation.x, 
                                                        current_pose.pose.orientation.y, 
                                                        current_pose.pose.orientation.z, 
                                                        current_pose.pose.orientation.w])
                euler = list(euler_tuple)
                euler[1] = euler[1] - 3.14/8.0
                quaternion = tf.transformations.quaternion_from_euler(euler[0], 
                                                                      euler[1], 
                                                                      euler[2])
                # Lift item to clear bin lip
                pose_target = geometry_msgs.msg.PoseStamped()
                pose_target.header.frame_id = 'base_footprint'
                rospy.loginfo("Lift")
                pose_target.pose.position.x = 0.431 #current_pose.pose.position.x
                pose_target.pose.position.y = object_pose.pose.position.y #current_pose.pose.position.y
                pose_target.pose.position.z = 1.57
                pose_target.pose.orientation.x = 0.98
                pose_target.pose.orientation.y = 0.039
                pose_target.pose.orientation.z = 0.18
                pose_target.pose.orientation.w = -0.020
                pose_target.pose.position.z = current_pose.pose.position.z 
                #pose_target.pose.position.x = current_pose.pose.position.x - 0.01 * i 

                # pose_target.header.frame_id = "base_footprint";
                # pose_target.pose.orientation.w = 1
                # pose_target.pose.position.x = grasp_dist + item_pose.position.x
                # pose_target.pose.position.y = robot_centered_offset #+ item_pose.position.y;
                # pose_target.pose.position.z = shelf_height + grasp_height + item_pose.position.z + lift_height
     
                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
            
                self._move_arm_ik.wait_for_service()
                try:
                    success_lift = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM).success
                except rospy.ServiceException:
                    rospy.sleep(1.0)
                    self._move_arm_ik.wait_for_service()

                    success_lift = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM).success
            if success_lift:
                rospy.loginfo("Lift success")
                break
            else:
                pose_target.pose.orientation = Quaternion() 
                success_lift = self._move_arm_ik(pose_target, MoveArmIkRequest().RIGHT_ARM).success
                rospy.loginfo("Lift attempt " + str(i) + " failed")
                continue


        if not success_lift and userdata.bin_id > "C":
            self._move_torso(self.torso_height_by_bin[userdata.bin_id], True)
        attempts = 5

        # Pull item out of bin
        success = False

        t = rospy.Time(0)
        position, quaternion = self._tf_listener.lookupTransform("shelf", "base_footprint", t)

        # find the target pose in robot coordinates
        target_in_shelf_frame = geometry_msgs.msg.PoseStamped(
            header=Header(frame_id='shelf'),
            pose=Pose(
                position=Point(x=-1.55,
                               y=position[1],
                               z=0.0),
                orientation=Quaternion(w=1, x=0, y=0, z=0)
            )
        )

        # Visualize target pose.
        marker = Marker()
        marker.header.frame_id = 'shelf'
        marker.header.stamp = rospy.Time().now()
        marker.ns = 'target_location'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = target_in_shelf_frame.pose
        marker.pose.position.z = 0.03 / 2
        marker.scale.x = 0.67
        marker.scale.y = 0.67
        marker.scale.z = 0.03
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.lifetime = rospy.Duration()

        rate = rospy.Rate(1)
        while self._markers.get_num_connections() == 0:
            rate.sleep()
        self._markers.publish(marker)

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        self._drive_to_pose.wait_for_service()
        self._drive_to_pose(pose=target_in_shelf_frame, linearVelocity=0.1, angularVelocity=0.1)

        if not success_lift:
            self._move_torso(self.torso_height_by_bin[userdata.bin_id] - 0.04, True)  


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

        #self._moveit_move_arm.wait_for_service()
        #self._moveit_move_arm(pose, 0.01, 0.01, 0, 'right_arm', False)


        if success:
            return outcomes.EXTRACT_ITEM_SUCCESS
        else:
            return outcomes.EXTRACT_ITEM_SUCCESS
