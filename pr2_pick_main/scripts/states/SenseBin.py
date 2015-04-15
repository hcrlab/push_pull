from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import CropShelfResponse
import outcomes
import rospy
import smach


class SenseBin(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_BIN'

    def __init__(self, tts, crop_shelf):
        """Constructor for this state.

        tts: The text-to-speech publisher.
        crop_shelf: The shelf cropping service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.SENSE_BIN_SUCCESS,
                outcomes.SENSE_BIN_NO_OBJECTS,
                outcomes.SENSE_BIN_FAILURE
            ],
            input_keys=['bin_id'],
            output_keys=['clusters']
        )
        self._tts = tts
        self._crop_shelf = crop_shelf

    def execute(self, userdata):
        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))
        # TODO(jstn): Move the head here.
        request = CropShelfRequest(cellID=userdata.bin_id)
        response = self._crop_shelf(request)
        response = CropShelfResponse()
        userdata.clusters = response.locations.clusters
        return outcomes.SENSE_BIN_SUCCESS
