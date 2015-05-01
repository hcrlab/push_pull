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
            output_keys=['clusters', 'target_cluster'])
        self._tts = tts
        self._crop_shelf = crop_shelf
        self._markers = markers
        self._tuck_arms = kwargs['tuck_arms']
        self._get_item_descriptor = kwargs['get_item_descriptor']
        self._classify_target_item = kwargs['classify_target_item']

    def execute(self, userdata):
        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))

        rospy.loginfo('Expecting target: {}, items: {}'.format(
            userdata.current_target, userdata.current_bin_items))
        self._tuck_arms.wait_for_service()
        self._tuck_arms(tuck_left=True, tuck_right=True)
        # If the arms are already tucked (usually true), then give some
        # time for the point cloud to update.
        rospy.sleep(5)

        # Get clusters
        request = CropShelfRequest(cellID=userdata.bin_id)
        self._crop_shelf.wait_for_service()
        response = self._crop_shelf(request)
        clusters = response.locations.clusters
        userdata.clusters = clusters
        rospy.loginfo('[SenseBin] Found {} clusters.'.format(
            len(response.locations.clusters)))
        if len(clusters) == 0:
            rospy.logerr('[SenseBin]: No clusters found!')
            return outcomes.SENSE_BIN_FAILURE

        descriptors = []
        for i, cluster in enumerate(clusters):
            # Publish visualization
            points = pc2.read_points(cluster.pointcloud,
                                     skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
            if len(point_list) == 0:
                rospy.logwarn('[SenseBin]: Cluster with 0 points returned!')
                continue
            viz.publish_cluster(self._markers, point_list,
                                'bin_{}'.format(userdata.bin_id),
                                'bin_{}_items'.format(userdata.bin_id), i)

            # Get descriptor
            self._get_item_descriptor.wait_for_service()
            response = self._get_item_descriptor(cluster=cluster)
            descriptors.append(response.descriptor)

        # Classify which cluster is the target item.
        if len(descriptors) == 0:
            rospy.logerr('[SenseBin]: No descriptors found!')
            return outcomes.SENSE_BIN_FAILURE
        self._classify_target_item.wait_for_service()
        response = self._classify_target_item(
            descriptors=descriptors,
            target_item=userdata.current_target,
            all_items=userdata.current_bin_items)
        index = response.target_item_index
        userdata.target_cluster = clusters[index]
        rospy.loginfo('Classified cluster #{} as target item ({} confidence)'.format(
            index, response.confidence
        ))

        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')
        return outcomes.SENSE_BIN_SUCCESS
