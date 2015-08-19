from bin_data import BinData
from pr2_pick_contest import PickingStrategy
from pr2_pick_main import handle_service_exceptions
import outcomes
import rospy
import smach
from pr2_pick_main.web_interface import WebInterface
from PushPullActions import RepositionAction
import rospkg
import json
import copy


class UpdatePlan(smach.State):
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
            input_keys=['debug', 'current_trial_num'],
            output_keys=['current_trial', 'current_trial_num']
        )
	
        self._tts = services['tts']
        self._interface = WebInterface()

        rospack = rospkg.RosPack()
        params_file = str(rospack.get_path('pr2_pick_contest')) + '/config/experiment_params.json' 

        with open(params_file) as data_file:    
            self._experiment_params = json.load(data_file)
        self._trials = self.generate_trials()
        #trial_num = raw_input("Enter the number of the last trial performed. Or enter -1 if this is the first trial:")
        self._trial_num = -1
        self._total_trials = len(self._trials)

    def generate_trials(self):
        trials = []
        actions = RepositionAction.get_all_actions()

        for item in self._experiment_params["items"]:
            for position in item["positions"]:
                for orientation in item["orientations"]:
                    for action in item["actions"]:
                        param_dict = item["action_params"][actions[action]]
                        if param_dict.keys():
                            param_lists = []
                            for key in param_dict:
                                param_lists.append(param_dict[key])
                                param_permutations = []

                            for params in itertools.product(*param_lists):

                                #for param in param_dict[key]:
                                trial = {}
                                trial["position"] = position
                                trial["orientation"] = orientation
                                trial["action"] = action
                                trial["item_name"] = item["item_name"]
                                trial["action_params"] = {}
                                trial["action_params"][actions[action]] = {}
                                for i in range(len(params)): 
                                    trial["action_params"][actions[action]][param_dict.keys()[i]] = params[i]
                                trials.append(copy.copy(trial))
                        else:
                            trial = {}
                            trial["position"] = position
                            trial["orientation"] = orientation
                            trial["action"] = action
                            trial["item_name"] = item["item_name"]
                            trials.append(copy.copy(trial))
        return trials  

    @handle_service_exceptions(outcomes.UPDATE_PLAN_FAILURE)
    def execute(self, userdata):

        self._trial_num += 1
        userdata.current_trial_num = self._trial_num
        rospy.loginfo('Trial number ' + str(self._trial_num) + ' out of ' + str(self._total_trials))
        userdata.current_trial = self._trials[self._trial_num]
        return outcomes.UPDATE_PLAN_NEXT_OBJECT

