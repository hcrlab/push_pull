from bin_data import BinData
from pr2_pick_contest import PickingStrategy
from pr2_pick_main import handle_service_exceptions
import outcomes
import rospy
import smach


class UpdatePlanExperiment(smach.State):
    """Decides on the next item to pick.

    Its preference is to go from bottom to top, then from left to right.
    It prefers trying bins that haven't been attempted before. If all the bins
    have been attempted, then it will try going to bins for which the attempt
    failed. It will do this until there are no more items to pick.
    """
    name = 'UPDATE_PLAN'

    def __init__(self, **kwargs):
        """Constructor for this state.
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
            output_keys=['output_bin_data', 'next_bin', 'next_target', 'next_bin_items', 'previous_item']
        )
	
        self._tts = kwargs['tts']
        #self._get_items = kwargs['get_items']
        #self._get_target_items = kwargs['get_target_items']
        self._lookup_item = kwargs['lookup_item']
        # How often this state has been run since the last time we relocalized
        # the shelf.
        self._calls_since_shelf_localization = 0
        #self._strategy = PickingStrategy(self._get_items, self._get_target_items, self._lookup_item)
        #plan = self._strategy.get_plan_by_expected_value()
        self._preferred_order = ["K"]
        #plan_string = '\n'.join(['{} {}'.format(letter, value) for letter, value in plan])
        #rospy.loginfo('Picking plan:\n{}'.format(plan_string))

    def check_all_visited(self, bin_data):
        all_visited = True
        for bin_id in self._preferred_order:
            if not bin_data[bin_id].visited:
                all_visited = False
                break 
        return all_visited

    @handle_service_exceptions(outcomes.UPDATE_PLAN_FAILURE)
    def execute(self, userdata):
        rospy.loginfo('Updating plan.')
        self._tts.publish('Updating plan.')
        rospy.loginfo("Calls since : " + str(self._calls_since_shelf_localization))
	items = ["highland_6539_self_stick_notes", "crayola_64_ct"] 
	userdata.previous_item = ""

        # If all bins have been visited, reset the visit states.
        # all_visited = self.check_all_visited(userdata.bin_data)
        # if all_visited:
        #     for bin_id in self._preferred_order:
        #         bin_data = userdata.bin_data.copy()
        #         bin_data[bin_id] = bin_data[bin_id]._replace(visited=False)
        #         userdata.output_bin_data = bin_data
	if(self._calls_since_shelf_localization == len(items)):
		self._calls_since_shelf_localization = 0
        bin_id = "K"
        userdata.next_bin = bin_id
        bin_data = userdata.bin_data.copy()
        bin_data[bin_id] = bin_data[bin_id]._replace(visited=True)
        userdata.output_bin_data = bin_data
        userdata.next_target = items[self._calls_since_shelf_localization ]
        userdata.next_bin_items = items[self._calls_since_shelf_localization]
        
        self._calls_since_shelf_localization += 1

        return outcomes.UPDATE_PLAN_NEXT_OBJECT

     