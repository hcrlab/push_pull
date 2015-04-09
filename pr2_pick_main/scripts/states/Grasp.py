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

    def __init__(self, set_grippers):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id']
        )

        self._set_grippers = set_grippers

    def execute(self, userdata):


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
        robot_centered_offset = 0 
        pre_grasp_dist = 0.20
        grasp_dist = 0.35
        grasp_height = 0.05

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


        # Pose in front of bin

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1;
        pose_target.position.x = pre_grasp_dist + item_pose.position.x;
        pose_target.position.y = robot_centered_offset + item_pose.position.y;
        pose_target.position.z = shelf_height + grasp_height + item_pose.position.z;
            
        group.set_pose_target(pose_target)
        plan = group.plan()
        group.go(wait=True)

        # Open Hand

        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(True, True)

        # Move into bin

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1;
        pose_target.position.x = grasp_dist + item_pose.position.x;
        pose_target.position.y = robot_centered_offset + item_pose.position.y;
        pose_target.position.z = shelf_height + grasp_height + item_pose.position.z;

        group.set_pose_target(pose_target)
        plan = group.plan()
        group.go(wait=True)

        # Close hand

        self._set_grippers.wait_for_service()
        grippers_open = self._set_grippers(False, False)



        return outcomes.GRASP_SUCCESS
