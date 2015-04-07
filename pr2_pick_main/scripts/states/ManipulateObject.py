import outcomes
import rospy
import smach


class ManipulateObject(smach.State):
    """Manipulates the target object (grasp, scoop, etc.)
    """
    name = 'MANIPULATE_OBJECT'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.MANIPULATE_OBJECT_SUCCESS,
                outcomes.MANIPULATE_OBJECT_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Manipulating object in bin {}'.format(userdata.bin_id))
        return outcomes.MANIPULATE_OBJECT_SUCCESS
