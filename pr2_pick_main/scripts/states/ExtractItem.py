import outcomes
import rospy
import smach


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
        return outcomes.EXTRACT_ITEM_SUCCESS
