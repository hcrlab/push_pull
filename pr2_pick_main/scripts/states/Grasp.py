import outcomes
import rospy
import smach
import moveit_commander
import geometry_msgs.msg

import json
import os
from pr2_pick_manipulation.srv import SetGrippers


class Grasp(smach.State):
    """Grasps an item in the bin.
    """
    name = 'GRASP'

    def __init__(self, set_grippers, tuck_arms, moveit_move_arm):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id']
        )

        self._set_grippers = set_grippers
        self._tuck_arms = tuck_arms
        self._moveit_move_arm = moveit_move_arm

    def execute(self, userdata):
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(False, False)


        # Get fake object locations

        current_dir = os.path.dirname(__file__)
        relative_path = "../../config/milestone_1_fake_object_locations.json"
        file_path = os.path.join(current_dir, relative_path)

        with open(file_path) as config_file:
            object_data = json.load(config_file)


        rospy.loginfo('Grasping item in bin {}'.format(userdata.bin_id))

        robot = moveit_commander.RobotCommander()

        group = moveit_commander.MoveGroupCommander("right_arm")


        # Parameters

        # Assuming MoveToBin centers itself on the leftmost wall of each bin 
        # If this is not true, set offset
        robot_centered_offset = -0.3 
        pre_grasp_dist = 0.40
        grasp_dist = 0.55
        grasp_height = 0.02
        pre_grasp_height = grasp_height + 0.01

        shelf_height_a_c = 1.55
        shelf_height_d_f = 1.32
        shelf_height_g_i = 1.09
        shelf_height_j_l = 0.84

        shelf_heights = {"A": shelf_height_a_c,
                            "B": shelf_height_a_c,
                            "C": shelf_height_a_c,
                            "D": shelf_height_d_f,
                            "E": shelf_height_d_f,
                            "F": shelf_height_d_f,
                            "G": shelf_height_g_i,
                            "H": shelf_height_g_i,
                            "I": shelf_height_g_i,
                            "J": shelf_height_j_l,
                            "K": shelf_height_j_l,
                            "L": shelf_height_j_l}

        shelf_height = shelf_heights[userdata.bin_id]
        
        # Get object info from work order

        item_pose = geometry_msgs.msg.Pose()

        for shelf_bin in object_data["work_order"]:
            if (shelf_bin["bin"] == "bin_" + str(userdata.bin_id) ):
                item_pose.position.x = shelf_bin["pose"]["x"]
                item_pose.position.y = shelf_bin["pose"]["y"]
                item_pose.position.z = shelf_bin["pose"]["z"]
                break

        # Center Arm

        rospy.loginfo("Center Arm")
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = 0.3135;
        pose.pose.position.y = -0.4665;
        pose.pose.position.z = 0.6905;
        pose.pose.orientation.x = -0.7969;
        pose.pose.orientation.y = 0.2719;
        pose.pose.orientation.z = -0.4802;
        pose.pose.orientation.w = -0.2458;

        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose)

        #group.set_pose_target(pose)
        #plan = group.plan()
        #group.go(wait=True)

        # Pose in front of bin

        rospy.loginfo("Pre-grasp:")
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.pose.orientation.w = 1
        pose_target.pose.position.x = pre_grasp_dist + item_pose.position.x
        pose_target.pose.position.y = robot_centered_offset #+ item_pose.position.y
        pose_target.pose.position.z = shelf_height + grasp_height + pre_grasp_height + item_pose.position.z
        
        """
        pose_target.position.x = 0.4107244589870565;
        pose_target.position.y = -0.3012263456842431;
        pose_target.position.z = 0.8830748767797871;
        pose_target.orientation.x = 0.9991422259206257;
        pose_target.orientation.y = -0.012209807030137479;
        pose_target.orientation.z = 0.019426512370710816;
        pose_target.orientation.w = 0.03447236011320723;
        """
        rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
        rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
        
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose_target)

       #group.set_pose_target(pose_target)
        #plan = group.plan()
        #group.go(wait=True)

        # Open Hand

        rospy.loginfo("Open Hand")
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(False, True)

        # Move into bin

        rospy.loginfo("Grasp")
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.pose.orientation.w = 1
        pose_target.pose.position.x = grasp_dist + item_pose.position.x
        pose_target.pose.position.y = robot_centered_offset #+ item_pose.position.y;
        pose_target.pose.position.z = shelf_height + grasp_height + item_pose.position.z
 
        """
        pose_target.position.x = 0.5481379095128065;
        pose_target.position.y = -0.3056166159947833;
        pose_target.position.z = 0.8643800971552054;
        pose_target.orientation.x = 0.999399420369509;
        pose_target.orientation.y = -0.010187825366838377;
        pose_target.orientation.z = 0.030767088106129725;
        pose_target.orientation.w = 0.012263485183839965
        """

        rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
        rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                
        self._moveit_move_arm.wait_for_service()
        self._moveit_move_arm(pose_target)

        #group.set_pose_target(pose_target)
        #plan = group.plan()
        #group.go(wait=True)

        # Close hand

        rospy.loginfo("Close Hand")
        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(False, False)



        return outcomes.GRASP_SUCCESS
