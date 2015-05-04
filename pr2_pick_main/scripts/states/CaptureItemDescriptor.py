from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import CropShelfResponse
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import outcomes
import rospy
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
            input_keys=['bin_id'],
            output_keys=['clusters'])
        self._tts = kwargs['tts']
        self._crop_shelf = kwargs['crop_shelf']
        self._markers = markers
        self._tuck_arms = kwargs['tuck_arms']
        self._get_item_descriptor = kwargs['get_item_descriptor']

    def execute(self, userdata):
        rospy.loginfo(
            'Capturing item descriptor in bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))
        rospy.sleep(5)
        self._tuck_arms.wait_for_service()
        self._tuck_arms(tuck_left=True, tuck_right=True)
        request = CropShelfRequest(cellID=userdata.bin_id)
        response = self._crop_shelf(request)
        userdata.clusters = response.locations.clusters
        rospy.loginfo('[CaptureItemDescriptor] Found {} clusters.'.format(
            len(response.locations.clusters)))
        if len(response.locations.clusters) != 1:
            raw_input('Need exactly 1 item. Press enter to try again: ')

        cluster = response.locations.clusters[0]
        points = pc2.read_points(cluster.pointcloud, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        viz.publish_cluster(self._markers, point_list,
                            'bin_{}'.format(userdata.bin_id),
                            'bin_{}_items'.format(userdata.bin_id), 0)

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
            bbox_dimensions.z, 0.33, 0.69, 0.31, 0.25, 1234)
        viz.publish_pose(self._markers, bbox_pose, 1, 0, 0, 1, 1234)

        action = None
        while action is None:
            action = raw_input('Capture [a]nother or [d]one?: ')
            if action == 'a':
                return outcomes.CAPTURE_ITEM_NEXT
            elif action == 'd':
                return outcomes.CAPTURE_ITEM_DONE
            else:
                action = None
