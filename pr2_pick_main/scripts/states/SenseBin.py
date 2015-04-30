from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import CropShelfResponse
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import outcomes
import rospy
import smach
import visualization as viz


class SenseBin(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_BIN'

    def __init__(self, tts, crop_shelf, markers, **kwargs):
        """Constructor for this state.

        tts: The text-to-speech publisher.
        crop_shelf: The shelf cropping service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.SENSE_BIN_SUCCESS, outcomes.SENSE_BIN_NO_OBJECTS,
                      outcomes.SENSE_BIN_FAILURE],
            input_keys=['bin_id', 'debug', 'current_target',
                        'current_bin_items'],
            output_keys=['clusters'])
        self._tts = tts
        self._crop_shelf = crop_shelf
        self._markers = markers
        self._tuck_arms = kwargs['tuck_arms']

    def execute(self, userdata):
        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))

        rospy.loginfo('Expecting target: {}, items: {}'.format(
            userdata.current_target, userdata.current_bin_items))
        self._tuck_arms.wait_for_service()
        self._tuck_arms(tuck_left=True, tuck_right=True)
        # TODO(jstn): Move the head here.
        request = CropShelfRequest(cellID=userdata.bin_id)
        response = self._crop_shelf(request)
        userdata.clusters = response.locations.clusters
        rospy.loginfo('[SenseBin] Found {} clusters.'.format(
            len(response.locations.clusters)))
        for i, cluster in enumerate(response.locations.clusters):
            points = pc2.read_points(cluster.pointcloud,
                                     skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z in points]
            viz.publish_cluster(self._markers, point_list,
                                'bin_{}'.format(userdata.bin_id),
                                'bin_{}_items'.format(userdata.bin_id), i)

        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')
        return outcomes.SENSE_BIN_SUCCESS
