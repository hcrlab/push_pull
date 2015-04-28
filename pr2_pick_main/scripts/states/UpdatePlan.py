from bin_data import BinData
import outcomes
import rospy
import smach


class UpdatePlan(smach.State):
    """Decides on the next item to pick.

    Its preference is to go from bottom to top, then from left to right.
    It prefers trying bins that haven't been attempted before. If all the bins
    have been attempted, then it will try going to bins for which the attempt
    failed. It will do this until there are no more items to pick.
    """
    name = 'UPDATE_PLAN'

    def __init__(self, **kwargs):
        """Constructor for this state.

        Args:
          tts: The text to speech publisher.
        """
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.UPDATE_PLAN_NEXT_OBJECT,
                outcomes.UPDATE_PLAN_RELOCALIZE_SHELF,
                outcomes.UPDATE_PLAN_NO_MORE_OBJECTS,
                outcomes.UPDATE_PLAN_FAILURE
            ],
            input_keys=['bin_data'],
            output_keys=['output_bin_data', 'next_bin']
        )
        self._tts = kwargs['tts']
        self._preferred_order = 'JKLGHIDEFABC'
        # How often this state has been run since the last time we relocalized
        # the shelf.
        self._calls_since_shelf_localization = 0

    def execute(self, userdata):
        rospy.loginfo('Updating plan.')
        self._tts.publish('Updating plan.')

        if self._calls_since_shelf_localization == 1:
            self._calls_since_shelf_localization = 0
            return outcomes.UPDATE_PLAN_RELOCALIZE_SHELF
        else:
            self._calls_since_shelf_localization += 1

        for bin_id in self._preferred_order:
            if not userdata.bin_data[bin_id].visited:
                userdata.next_bin = bin_id
                bin_data = userdata.bin_data.copy()
                bin_data[bin_id] = bin_data[bin_id]._replace(visited=True)
                userdata.output_bin_data = bin_data
                return outcomes.UPDATE_PLAN_NEXT_OBJECT

        # take another pass at all the bins that did not succeed
        for bin_id in self._preferred_order:
            if not userdata.bin_data[bin_id].succeeded:
                userdata.next_bin = bin_id
                return outcomes.UPDATE_PLAN_NEXT_OBJECT

        return outcomes.UPDATE_PLAN_NO_MORE_OBJECTS
