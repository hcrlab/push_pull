from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import SegmentItemsRequest
from sensor_msgs.msg import PointCloud2
import outcomes
import rospy
import sensor_msgs.point_cloud2 as pc2
import smach
import visualization as viz


class CaptureItemDescriptor(smach.State):
    """Performs sensing on a bin and records the data.
    """
    name = 'CAPTURE_ITEM_DESCRIPTOR'

    def __init__(self, markers, **kwargs):
        """Constructor for this state.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.CAPTURE_ITEM_NEXT, outcomes.CAPTURE_ITEM_DONE],
            input_keys=['bin_id', 'current_bin_items'],
            output_keys=['clusters'])
        self._tts = kwargs['tts']
        self._crop_shelf = kwargs['crop_shelf']
        self._segment_items = kwargs['segment_items']
        self._markers = markers
        self._tuck_arms = kwargs['tuck_arms']
        self._get_item_descriptor = kwargs['get_item_descriptor']

    @handle_service_exceptions(outcomes.CAPTURE_ITEM_DONE)
    def execute(self, userdata):
        rospy.loginfo(
            'Capturing item descriptor in bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))
        rospy.sleep(5)
        self._tuck_arms.wait_for_service()
        self._tuck_arms(tuck_left=False, tuck_right=False)

        # Crop shelf.
        crop_request = CropShelfRequest(cellID=userdata.bin_id)
        self._crop_shelf.wait_for_service()
        crop_response = self._crop_shelf(crop_request)

        # Segment items.
        segment_request = SegmentItemsRequest(cloud=crop_response.cloud, items=userdata.current_bin_items)
        self._segment_items.wait_for_service()
        segment_response = self._segment_items(segment_request)
        clusters = segment_response.clusters.clusters
        userdata.clusters = clusters
        rospy.loginfo('[CaptureItemDescriptor] Found {} clusters.'.format(
            len(clusters)))

        for i, cluster in enumerate(clusters):
            points = pc2.read_points(cluster.pointcloud, skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
            if len(point_list) == 0:
                rospy.logwarn('Skipping cluster of size 0')
                continue
            viz.publish_cluster(self._markers, point_list,
                                'bin_{}'.format(userdata.bin_id),
                                'bin_{}_items'.format(userdata.bin_id), i)

            self._get_item_descriptor.wait_for_service()
            descriptor = self._get_item_descriptor(cluster).descriptor

            rospy.loginfo('Color histogram ({} bins):\n{}'.format(
                descriptor.histogram.num_bins, descriptor.histogram.histogram))

            bounding_box = descriptor.planar_bounding_box
            bbox_pose = bounding_box.pose
            bbox_dimensions = bounding_box.dimensions
            rospy.loginfo('Bounding box centroid: {}'.format(bbox_pose))
            rospy.loginfo('Bounding box dimensions: {}'.format(bbox_dimensions))
            viz.publish_bounding_box(
                self._markers, bbox_pose, bbox_dimensions.x, bbox_dimensions.y,
                bbox_dimensions.z, 0.33, 0.69, 0.31, 0.25, 1234 + i)
            viz.publish_pose(self._markers, bbox_pose, 1, 0, 0, 1, 1234 + i)

        action = None
        while action is None:
            action = raw_input('Capture [a]nother or [d]one?: ')
            if action == 'a':
                return outcomes.CAPTURE_ITEM_NEXT
            elif action == 'd':
                return outcomes.CAPTURE_ITEM_DONE
            else:
                action = None
