import rospy
import smach
import outcomes

class VerifyGrasp(smach.State):
    ''' Grasps an item in the bin. '''
    name = 'VERIFY_GRASP'

    def __init__(self, get_grippers, **kwargs):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.VERIFY_GRASP_SUCCESS,
                outcomes.VERIFY_GRASP_FAILURE,
                outcomes.VERIFY_GRASP_RETRY
            ],
            input_keys=['bin_id', 'debug', 'bin_data'],
            output_keys=['output_bin_data']
        )

        self._get_grippers = get_grippers

    def execute(self, userdata):
    	# update bin_data
    	output_bin_data = userdata.bin_data.copy()
    	bin_id = userdata.bin_id
    	output_bin_data[bin_id] = output_bin_data[bin_id]._replace(
    		attempts_remaining=output_bin_data[bin_id].attempts_remaining-1)
    	userdata.output_bin_data = output_bin_data

    	# check if grasp succeeded
    	gripper_states = self._get_grippers()
    	# TODO: get this information from JSON file
    	thin_object = False 
    	# for thin objects, a closed gripper is consistent with success
    	grasp_succeeded = gripper_states.right_open or thin_object

    	# decide what state to go to next
    	if grasp_succeeded:
    		# go to DropOffItem
    		return outcomes.VERIFY_GRASP_SUCCESS
    	else:
    		# check if there are attempts remaining
    		if output_bin_data[bin_id].attempts_remaining == 0:
    			return outcomes.VERIFY_GRASP_FAILURE
    		else:
    			return outcomes.VERIFY_GRASP_RETRY