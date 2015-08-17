from bin_data import BinData
from pr2_pick_contest import PickingStrategy
from pr2_pick_main import handle_service_exceptions
import outcomes
import rospy
import smach
import json
import copy
import rospkg


class UpdatePlanExperiment(smach.State):
    """Decides on the next item to pick.

    Its preference is to go from bottom to top, then from left to right.
    It prefers trying bins that haven't been attempted before. If all the bins
    have been attempted, then it will try going to bins for which the attempt
    failed. It will do this until there are no more items to pick.
    """
    name = 'UPDATE_PLAN'

    def __init__(self, **services):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.UPDATE_PLAN_NEXT_OBJECT,
                outcomes.UPDATE_PLAN_FAILURE
            ],
            input_keys=['bin_data'],
            output_keys=['output_bin_data', 'next_bin', 'next_target', 'next_bin_items', 'previous_item', 'current_trial']
        )
	
        self._tts = services['tts']
        self._lookup_item = services['lookup_item']
        self._calls_since_shelf_localization = 0
        self._preferred_order = ["K"]

        rospack = rospkg.RosPack()
        params_file = str(rospack.get_path('pr2_pick_contest')) + '/config/experiment_params.json' 

        with open(params_file) as data_file:    
            self._experiment_params = json.load(data_file)
        self._trials = self.generate_trials()
        self._trial_num = -1
        self._total_trials = len(self._trials)

    def generate_trials(self):
        trials = []
        actions = ["front_center_push", "front_side_push", "front_side_push",
                    "push_full_contact" , "push_full_contact", "push_point_contact", "push_point_contact",
                    "top_pull", "top_sideways_pull", "top_sideways_pull"]
        for item in self._experiment_params["items"]:
            for position in item["positions"]:
                for orientation in item["orientations"]:
                    for action in item["actions"]:
                        param_dict = item["action_params"][actions[action]]
                        if param_dict.keys():
                            for key in param_dict:
                                for param in param_dict[key]:
                                    trial = {}
                                    trial["position"] = position
                                    trial["orientation"] = orientation
                                    trial["action"] = action
                                    trial["item_name"] = item["item_name"]
                                    trial["action_params"] = {}
                                    trial["action_params"][actions[action]] = {} 
                                    trial["action_params"][actions[action]][key] = param
                                    trials.append(copy.copy(trial))
                        else:
                            trial = {}
                            trial["position"] = position
                            trial["orientation"] = orientation
                            trial["action"] = action
                            trial["item_name"] = item["item_name"]
                            trials.append(copy.copy(trial))
        return trials  


    def check_all_visited(self, bin_data):
        all_visited = True
        for bin_id in self._preferred_order:
            if not bin_data[bin_id].visited:
                all_visited = False
                break 
        return all_visited

    @handle_service_exceptions(outcomes.UPDATE_PLAN_FAILURE)
    def execute(self, userdata):
        self._trial_num+=1
        print "Trial number " + str(self._trial_num) + " out of " + str(self._total_trials)
        userdata.current_trial = self._trials[self._trial_num]
        rospy.loginfo('Updating plan.')
        self._tts.publish('Updating plan.')
        rospy.loginfo("Calls since : " + str(self._calls_since_shelf_localization))
        items = ["highland_6539_self_stick_notes", "crayola_64_ct"] 
        userdata.previous_item = ""

        #if(self._calls_since_shelf_localization == len(items)):
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

     
