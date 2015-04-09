import outcomes
import rospy
import smach
import moveit_commander
import geometry_msgs.msg


class Grasp(smach.State):
    """Grasps an item in the bin.
    """
    name = 'GRASP'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_SUCCESS,
                outcomes.GRASP_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Grasping item in bin {}'.format(userdata.bin_id))

        robot = moveit_commander.RobotCommander()

        group = moveit_commander.MoveGroupCommander("right_arm")

        rospy.wait_for_service('gripper_service')
        self._set_grippers = rospy.ServiceProxy('gripper_service', SetGrippers)

        # Poses for each bin

        bin_dict = {}
        bin_dict["A"]["pre-grasp"] = geometry_msgs.msg.Pose()
        bin_dict["A"]["pre-grasp"].orientation.w = 1;
        bin_dict["A"]["pre-grasp"].position.x = 0.2;
        bin_dict["A"]["pre-grasp"].position.y = 0.0;
        bin_dict["A"]["pre-grasp"].position.z = 1.9;

        bin_dict["A"]["grasp"] = geometry_msgs.msg.Pose()
        bin_dict["A"]["grasp"].orientation.w = 1;
        bin_dict["A"]["grasp"].position.x = 0.35;
        bin_dict["A"]["grasp"].position.y = 0.0;
        bin_dict["A"]["grasp"].position.z = 1.9;

        bin_dict["D"]["pre-grasp"] = geometry_msgs.msg.Pose()
        bin_dict["D"]["pre-grasp"].orientation.w = 1;
        bin_dict["D"]["pre-grasp"].position.x = 0.2;
        bin_dict["D"]["pre-grasp"].position.y = 0.0;
        bin_dict["D"]["pre-grasp"].position.z = 1.6;

        bin_dict["D"]["grasp"] = geometry_msgs.msg.Pose()
        bin_dict["D"]["grasp"].orientation.w = 1;
        bin_dict["D"]["grasp"].position.x = 0.35;
        bin_dict["D"]["grasp"].position.y = 0.0;
        bin_dict["D"]["grasp"].position.z = 1.6;

        bin_dict["G"]["pre-grasp"] = geometry_msgs.msg.Pose()
        bin_dict["G"]["pre-grasp"].orientation.w = 1;
        bin_dict["G"]["pre-grasp"].position.x = 0.2;
        bin_dict["G"]["pre-grasp"].position.y = 0.0;
        bin_dict["G"]["pre-grasp"].position.z = 1.3;

        bin_dict["G"]["grasp"] = geometry_msgs.msg.Pose()
        bin_dict["G"]["grasp"].orientation.w = 1;
        bin_dict["G"]["grasp"].position.x = 0.35;
        bin_dict["G"]["grasp"].position.y = 0.0;
        bin_dict["G"]["grasp"].position.z = 1.3;

        bin_dict["J"]["pre-grasp"] = geometry_msgs.msg.Pose()
        bin_dict["J"]["pre-grasp"].orientation.w = 1;
        bin_dict["J"]["pre-grasp"].position.x = 0.2;
        bin_dict["J"]["pre-grasp"].position.y = 0.0;
        bin_dict["J"]["pre-grasp"].position.z = 1.0;

        bin_dict["J"]["grasp"] = geometry_msgs.msg.Pose()
        bin_dict["J"]["grasp"].orientation.w = 1;
        bin_dict["J"]["grasp"].position.x = 0.35;
        bin_dict["J"]["grasp"].position.y = 0.0;
        bin_dict["J"]["grasp"].position.z = 1.0;

        bin_dict["B"] = bin_dict["C"] = bin_dict["A"]

        bin_dict["E"] = bin_dict["F"] = bin_dict["D"]

        bin_dict["H"] = bin_dict["I"] = bin_dict["G"]

        bin_dict["K"] = bin_dict["L"] = bin_dict["J"]


        # Pose in front of bin
            
        group.set_pose_target(bin_dict[userdata.bin_id]["pre-grasp"])
        plan = group.plan()
        group.go(wait=True)

        # Open Hand

        grippers_open = self._set_grippers(True, True)

        # Move into bin

        group.set_pose_target(bin_dict[userdata.bin_id]["grasp"])
        plan = group.plan()
        group.go(wait=True)

        # Close hand

        grippers_open = self._set_grippers(False, False)



        return outcomes.GRASP_SUCCESS
