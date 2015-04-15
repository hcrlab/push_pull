import outcomes
import rospy
import smach
import moveit_commander
import geometry_msgs.msg
import tf
import json
import os
from pr2_pick_manipulation.srv import SetGrippers
from pr2_pick_manipulation.srv import GetPose
from pr2_pick_manipulation.srv import MoveArm
import rospkg

class Grasp(smach.State):
    """Grasps an item in the bin.
    """
    name = 'GRASP'

    def __init__(self, tts, set_grippers, tuck_arms, moveit_move_arm):
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
        self._tts = tts

        # Shelf heights

        self._shelf_height_a_c = 1.55
        self._shelf_height_d_f = 1.32
        self._shelf_height_g_i = 1.09
        self._shelf_height_j_l = 0.84

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

    def execute(self, userdata):
        self._tts.publish('Grasping item')
        self._tuck_arms.wait_for_service()
        tuck_success = self._tuck_arms(False, False)


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

        transformed_item_pose = tf.transformPose("base_footprint", item_pose)


        rospy.loginfo('Grasping item in bin {}'.format(userdata.bin_id))

        #robot = moveit_commander.RobotCommander()

        #group = moveit_commander.MoveGroupCommander("right_arm")

        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object("shelf")

    

        shelf_pose = geometry_msgs.msg.PoseStamped()
        shelf_pose.header.frame_id = "/shelf"
        shelf_pose.pose.position.x = 0.0
        shelf_pose.pose.position.y = 0.0
        shelf_pose.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(1.57,0,1.57)
        shelf_pose.pose.orientation.x = q[0]
        shelf_pose.pose.orientation.y = q[1]
        shelf_pose.pose.orientation.z = q[2]
        shelf_pose.pose.orientation.w = q[3]
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # list all packages, equivalent to rospack list
        #rospack.list_pkgs() 

        # get the file path for rospy_tutorials
        path = rospack.get_path('pr2_pick_contest')


        
        shelf_mesh = path + "/config/kiva_pod/meshes/pod_lowres.stl" 
        #scene.add_mesh("shelf", shelf_pose, shelf_mesh)

        

        shelf_height = self._shelf_heights[userdata.bin_id]
        
        # Center Arm

        rospy.loginfo("Center Arm")
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "base_footprint";

        if userdata.bin_id > "F":
            pose.pose.position.x = 0.3135;
            pose.pose.position.y = -0.4665;
            pose.pose.position.z = 0.6905;
            pose.pose.orientation.x = -0.7969;
            pose.pose.orientation.y = 0.2719;
            pose.pose.orientation.z = -0.4802;
            pose.pose.orientation.w = -0.2458;
        elif userdata.bin_id > "C":
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
        self._moveit_move_arm(pose, 0, "right_arm")

        dist_to_palm = 0.077
        dist_to_fingertips = 0.18
        attempts = 3 
        
        success_pre_grasp = False

        for i in range(attempts):

            # Pose in front of bin

            rospy.loginfo("Pre-grasp:")
            pose_target = geometry_msgs.msg.PoseStamped()
            pose_target.header.frame_id = "base_footprint";

            if userdata.bin_id > "C":
                pose_target.pose.orientation.w = 1
                pose_target.pose.position.x = self._pre_grasp_dist + 0.01 * i 
                pose_target.pose.position.y = transformed_item_pose.pose.position.y
                if (transformed_item_pose.pose.position.z > (shelf_height + self._grasp_height)) and (transformed_item_pose.pose.position.z < (shelf_height + 0.15)):
                    pose_target.pose.position.z = transformed_item_pose.pose.position.z
                else:
                    pose_target.pose.position.z = shelf_height + self._grasp_height + self._pre_grasp_height
                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                
                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(pose_target, 0.001, 0.001, 0, "right_arm")
            else:
                #   pre  - Translation: [0.253, -0.277, 1.508]
                # - Rotation: in Quaternion [0.984, -0.013, 0.178, 0.028]
                #             in RPY [3.087, -0.359, -0.017]

                
                pose_target.pose.orientation.x = 0.984
                pose_target.pose.orientation.y = -0.013
                pose_target.pose.orientation.z = 0.178
                pose_target.pose.orientation.w = 0.028
                pose_target.pose.position.x = 0.243 + 0.01 * i 
                pose_target.pose.position.y = transformed_item_pose.pose.position.y
                pose_target.pose.position.z = 1.508

                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                
                self._moveit_move_arm.wait_for_service()
                success_pre_grasp = self._moveit_move_arm(pose_target, 0.01, 0.01, 0, "right_arm")

            if success_pre_grasp:
                # Open Hand

                rospy.loginfo("Open Hand")
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(False, True)

                break
            else:
                continue

        if not success_pre_grasp:
            return outcomes.GRASP_FAILURE

        success_grasp = False
            
        for i in range(int(dist_to_fingertips - dist_to_palm)):

            # Move into bin

            rospy.loginfo("Grasp")
            pose_target = geometry_msgs.msg.PoseStamped()
            pose_target.header.frame_id = "base_footprint";
            if userdata.bin_id > "C":
                pose_target.pose.orientation.w = 1
                pose_target.pose.position.x = transformed_item_pose.pose.position.x - dist_to_palm - 0.01 * i
                pose_target.pose.position.y = transformed_item_pose.pose.position.y
                if (transformed_item_pose.pose.position.z > (shelf_height + self._grasp_height)) and (transformed_item_pose.pose.position.z < (shelf_height + 0.15)):
                    pose_target.pose.position.z = transformed_item_pose.pose.position.z
                else:
                    pose_target.pose.position.z = shelf_height + self._grasp_height


                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                        
                self._moveit_move_arm.wait_for_service()
                success_grasp = self._moveit_move_arm(pose_target, 0.0001, 0.001, 0, "right_arm")

            else:
                # real - Translation: [0.431, -0.280, 1.570]
                # - Rotation: in Quaternion [0.996, -0.016, 0.080, 0.027]
                #     in RPY [3.090, -0.162, -0.028]
                pose_target.pose.orientation.x = 0.996
                pose_target.pose.orientation.y = -0.016
                pose_target.pose.orientation.z = 0.080
                pose_target.pose.orientation.w = 0.027
                pose_target.pose.position.x = 0.431 - 0.01 * i 
                pose_target.pose.position.y = transformed_item_pose.pose.position.y
                pose_target.pose.position.z = 1.570


                rospy.loginfo("pose x: " + str(pose_target.pose.position.x) + ", y: " + str(pose_target.pose.position.y) + ", z: " + str(pose_target.pose.position.z))
                rospy.loginfo("orientation x: " + str(pose_target.pose.orientation.x) + ", y: " + str(pose_target.pose.orientation.y) + ", z: " + str(pose_target.pose.orientation.z) + ", w: " + str(pose_target.pose.orientation.w))
                        
                self._moveit_move_arm.wait_for_service()
                success_grasp = self._moveit_move_arm(pose_target, 0.01, 0.01, 0, "right_arm")



            if success_grasp:
                # Close hand

                rospy.loginfo("Close Hand")
                self._set_grippers.wait_for_service()
                grippers_open = self._set_grippers(False, False)

                break
            else:
                continue



        if not success_grasp:
            return outcomes.GRASP_FAILURE
        else:
            return outcomes.GRASP_SUCCESS
