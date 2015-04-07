import outcomes
import rospy
import smach


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
        return outcomes.GRASP_SUCCESS
