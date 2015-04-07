import outcomes
import rospy
import smach


class DropOffObject(smach.State):
    """Deposits the object or objects into the order bin.
    """
    name = 'DROP_OFF_OBJECT'

    def __init__(self):
        smach.State.__init__(self,
            outcomes=[
                outcomes.DROP_OFF_OBJECT_SUCCESS,
                outcomes.DROP_OFF_OBJECT_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Dropping off object from bin {}'.format(userdata.bin_id))
        return outcomes.DROP_OFF_OBJECT_SUCCESS
