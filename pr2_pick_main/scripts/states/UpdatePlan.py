import outcomes
import rospy
import smach


class UpdatePlan(smach.State):
    """Decides on the next object to pick.
    """
    name = 'UPDATE_PLAN'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.UPDATE_PLAN_NEXT_OBJECT,
                outcomes.UPDATE_PLAN_NO_MORE_OBJECTS,
                outcomes.UPDATE_PLAN_FAILURE
            ],
            input_keys=['last_bin_id'],
            output_keys=['last_bin_id', 'bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Updating plan.')
        if userdata.last_bin_id is None:
            userdata.last_bin_id = -1

        if userdata.last_bin_id == 2:
            return outcomes.UPDATE_PLAN_NO_MORE_OBJECTS
        elif userdata.last_bin_id >= -1:
            bin_id = userdata.last_bin_id + 1
            userdata.bin_id = bin_id
            userdata.last_bin_id = bin_id
            return outcomes.UPDATE_PLAN_NEXT_OBJECT
        else:
            return outcomes.UPDATE_PLAN_FAILURE

