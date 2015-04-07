import outcomes
import rospy
import smach


class FindShelf(smach.State):
    """Localizes the shelf.
    """
    name = 'FIND_SHELF'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.FIND_SHELF_SUCCESS,
                outcomes.FIND_SHELF_FAILURE
            ]
        )

    def execute(self, userdata):
        rospy.loginfo('Finding shelf.')
        return outcomes.FIND_SHELF_SUCCESS
