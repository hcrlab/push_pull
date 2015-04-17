from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import CropShelfResponse
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
import outcomes
import random
import rospy
import smach


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
            outcomes=[
                outcomes.SENSE_BIN_SUCCESS,
                outcomes.SENSE_BIN_NO_OBJECTS,
                outcomes.SENSE_BIN_FAILURE
            ],
            input_keys=['bin_id', 'debug'],
            output_keys=['clusters']
        )
        self._tts = tts
        self._crop_shelf = crop_shelf
        self._markers = markers

    def execute(self, userdata):
        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))
        # TODO(jstn): Move the head here.
        request = CropShelfRequest(cellID=userdata.bin_id)
        response = self._crop_shelf(request)
        userdata.clusters = response.locations.clusters
        rospy.loginfo('[SenseBin] Found {} clusters.'.format(
            len(response.locations.clusters)))
        for i, cluster in enumerate(response.locations.clusters):
            points = pc2.read_points(cluster.pointcloud,
                                     field_names=['x', 'y', 'z'],
                                     skip_nans=True)
            marker = Marker()
            # TODO(jstn): Once the point clouds have the correct frame_id,
            # use them here.
            marker.header.frame_id = 'bin_{}'.format(userdata.bin_id)
            marker.header.stamp = rospy.Time().now()
            marker.ns = 'bin_{}_items'.format(userdata.bin_id)
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.color.r = random.random()
            marker.color.g = random.random()
            marker.color.b = random.random()
            marker.color.a = 1
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.lifetime = rospy.Duration()

            self._markers.publish(marker)

        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')
        return outcomes.SENSE_BIN_SUCCESS
