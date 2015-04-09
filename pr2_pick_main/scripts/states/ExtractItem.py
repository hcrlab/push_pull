import outcomes
import rospy
import smach
import moveit_commander
import geometry_msgs.msg


class ExtractItem(smach.State):
    """Extracts the target item from the bin.
    """
    name = 'EXTRACT_ITEM'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.EXTRACT_ITEM_SUCCESS,
                outcomes.EXTRACT_ITEM_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Extracting item in bin {}'.format(userdata.bin_id))

        robot = moveit_commander.RobotCommander()

        group = moveit_commander.MoveGroupCommander("right_arm")

        # Poses for each bin

        bin_dict = {}
        bin_dict["A"]["lift"] = geometry_msgs.msg.Pose()
        bin_dict["A"]["lift"].orientation.w = 1;
        bin_dict["A"]["lift"].position.x = 0.2;
        bin_dict["A"]["lift"].position.y = 0.0;
        bin_dict["A"]["lift"].position.z = 1.95;

        bin_dict["A"]["extract"] = geometry_msgs.msg.Pose()
        bin_dict["A"]["extract"].orientation.w = 1;
        bin_dict["A"]["extract"].position.x = 0.35;
        bin_dict["A"]["extract"].position.y = 0.0;
        bin_dict["A"]["extract"].position.z = 1.95;

        bin_dict["D"]["lift"] = geometry_msgs.msg.Pose()
        bin_dict["D"]["lift"].orientation.w = 1;
        bin_dict["D"]["lift"].position.x = 0.2;
        bin_dict["D"]["lift"].position.y = 0.0;
        bin_dict["D"]["lift"].position.z = 1.65;

        bin_dict["D"]["extract"] = geometry_msgs.msg.Pose()
        bin_dict["D"]["extract"].orientation.w = 1;
        bin_dict["D"]["extract"].position.x = 0.35;
        bin_dict["D"]["extract"].position.y = 0.0;
        bin_dict["D"]["extract"].position.z = 1.65;

        bin_dict["G"]["lift"] = geometry_msgs.msg.Pose()
        bin_dict["G"]["lift"].orientation.w = 1;
        bin_dict["G"]["lift"].position.x = 0.2;
        bin_dict["G"]["lift"].position.y = 0.0;
        bin_dict["G"]["lift"].position.z = 1.35;

        bin_dict["G"]["extract"] = geometry_msgs.msg.Pose()
        bin_dict["G"]["extract"].orientation.w = 1;
        bin_dict["G"]["extract"].position.x = 0.35;
        bin_dict["G"]["extract"].position.y = 0.0;
        bin_dict["G"]["extract"].position.z = 1.35;

        bin_dict["J"]["lift"] = geometry_msgs.msg.Pose()
        bin_dict["J"]["lift"].orientation.w = 1;
        bin_dict["J"]["lift"].position.x = 0.2;
        bin_dict["J"]["lift"].position.y = 0.0;
        bin_dict["J"]["lift"].position.z = 1.05;

        bin_dict["J"]["extract"] = geometry_msgs.msg.Pose()
        bin_dict["J"]["extract"].orientation.w = 1;
        bin_dict["J"]["extract"].position.x = 0.35;
        bin_dict["J"]["extract"].position.y = 0.0;
        bin_dict["J"]["extract"].position.z = 1.05;

        bin_dict["B"] = bin_dict["C"] = bin_dict["A"]

        bin_dict["E"] = bin_dict["F"] = bin_dict["D"]

        bin_dict["H"] = bin_dict["I"] = bin_dict["G"]

        bin_dict["K"] = bin_dict["L"] = bin_dict["J"]


        # Lift item to clear bin lip
            
        group.set_pose_target(bin_dict[userdata.bin_id]["lift"])
        plan = group.plan()
        group.go(wait=True)

        # Pull item out of bin

        group.set_pose_target(bin_dict[userdata.bin_id]["extract"])
        plan = group.plan()
        group.go(wait=True)




        return outcomes.EXTRACT_ITEM_SUCCESS
