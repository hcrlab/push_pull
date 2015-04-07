import outcomes
import rospy
import smach


class DropOffItem(smach.State):
    """Deposits the item into the order bin.
    """
    name = 'DROP_OFF_ITEM'

    def __init__(self):
        smach.State.__init__(self,
            outcomes=[
                outcomes.DROP_OFF_ITEM_SUCCESS,
                outcomes.DROP_OFF_ITEM_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Dropping off item from bin {}'.format(userdata.bin_id))
        return outcomes.DROP_OFF_ITEM_SUCCESS
