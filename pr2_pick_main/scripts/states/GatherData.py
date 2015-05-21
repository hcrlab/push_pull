from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception.msg import MultiItemCloud
from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import SegmentItemsRequest
from sensor_msgs.msg import PointCloud2
import outcomes
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import smach
import visualization as viz

CONST_ITEM_NAMES = [
    "oreo_mega_stuf",
    "champion_copper_plus_spark_plug",
    "expo_dry_erase_board_eraser",
    "kong_duck_dog_toy",
    "genuine_joe_plastic_stir_sticks",
    "munchkin_white_hot_duck_bath_toy",
    "crayola_64_ct",
    "mommys_helper_outlet_plugs",
    "sharpie_accent_tank_style_highlighters",
    "kong_air_dog_squeakair_tennis_ball",
    "stanley_66_052",
    "safety_works_safety_glasses",
    "dr_browns_bottle_brush",
    "laugh_out_loud_joke_book",
    "cheezit_big_original",
    "paper_mate_12_count_mirado_black_warrior",
    "feline_greenies_dental_treats",
    "elmers_washable_no_run_school_glue",
    "mead_index_cards",
    "rolodex_jumbo_pencil_cup",
    "first_years_take_and_toss_straw_cup",
    "highland_6539_self_stick_notes",
    "mark_twain_huckleberry_finn",
    "kyjen_squeakin_eggs_plush_puppies",
    "kong_sitting_frog_dog_toy"
]  # yapf: disable


class GatherData(smach.State):
    """Performs sensing on a bin and records the data.
    """
    name = 'GATHER_DATA'

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
        self._segment_items = kwargs['segment_items']
        self._markers = markers
        self._tuck_arms = kwargs['tuck_arms']
        self._get_item_descriptor = kwargs['get_item_descriptor']

        bag_file_path = '{}/bag/cell_pc_{}.bag'.format(
            os.path.expanduser('~'),
            int(rospy.Time.now().to_sec()))
        self._bag = rosbag.Bag(bag_file_path, 'w')
        rospy.loginfo('Saving data to {}'.format(bag_file_path))

    def prompt_for_labels(self):
        for i, item in enumerate(CONST_ITEM_NAMES):
            print '{}\t{}'.format(i, item)
        user_input = raw_input('Enter the item number or a space separated list of items: ')
        label_ints = [int(x) for x in user_input.split(' ') if x != '']
        labels = [CONST_ITEM_NAMES[i] for i in label_ints]
        return labels

    @handle_service_exceptions(outcomes.CAPTURE_ITEM_DONE)
    def execute(self, userdata):
        rospy.loginfo(
            'Gathering data in bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))
        rospy.sleep(5)
        self._tuck_arms.wait_for_service()
        self._tuck_arms(tuck_left=False, tuck_right=False)

        # Crop shelf.
        crop_request = CropShelfRequest(cellID=userdata.bin_id)
        self._crop_shelf.wait_for_service()
        crop_response = self._crop_shelf(crop_request)

        # Publish visualization.
        points = pc2.read_points(crop_response.cloud, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        if len(point_list) > 0:
            viz.publish_cluster(self._markers, point_list,
                            'bin_{}'.format(userdata.bin_id),
                            'bin_{}_items'.format(userdata.bin_id), 111)

            # Get labels.
            labels = self.prompt_for_labels()
            print 'You selected: ', labels
            print 'Verify that the point cloud looks right before saving.'
            
            # Save to bag file.
            action = raw_input('Does the data look good? [y]es/[n]o: ')
            if action == 'y':
                example = MultiItemCloud()
                example.cloud = crop_response.cloud
                example.labels = labels
                self._bag.write('cell_pc', example)

        else:
            rospy.logwarn('Point cloud was of size 0.')

        # Segment items.
        #segment_request = SegmentItemsRequest(cloud=crop_response.cloud)
        #self._segment_items.wait_for_service()
        #segment_response = self._segment_items(segment_request)
        #clusters = segment_response.clusters.clusters
        #userdata.clusters = clusters
        #rospy.loginfo(
        #    '[CaptureItemDescriptor] Found {} clusters.'.format(len(clusters)))

        #for i, cluster in enumerate(clusters):
        #    points = pc2.read_points(cluster.pointcloud, skip_nans=True)
        #    point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        #    if len(point_list) == 0:
        #        rospy.logwarn('Skipping cluster of size 0')
        #        continue
        #    viz.publish_cluster(self._markers, point_list,
        #                        'bin_{}'.format(userdata.bin_id),
        #                        'bin_{}_items'.format(userdata.bin_id), i)

        #    self._get_item_descriptor.wait_for_service()
        #    descriptor = self._get_item_descriptor(cluster).descriptor

        #    rospy.loginfo('Color histogram ({} bins):\n{}'.format(
        #        descriptor.histogram.num_bins, descriptor.histogram.histogram))

        #    bounding_box = descriptor.planar_bounding_box
        #    bbox_pose = bounding_box.pose
        #    bbox_dimensions = bounding_box.dimensions
        #    rospy.loginfo('Bounding box centroid: {}'.format(bbox_pose))
        #    rospy.loginfo('Bounding box dimensions: {}'.format(bbox_dimensions))
        #    viz.publish_bounding_box(self._markers, bbox_pose,
        #                             bbox_dimensions.x, bbox_dimensions.y,
        #                             bbox_dimensions.z, 0.33, 0.69, 0.31, 0.25,
        #                             1234 + i)
        #    viz.publish_pose(self._markers, bbox_pose, 1, 0, 0, 1, 1234 + i)

        action = None
        while action is None:
            action = raw_input('Capture [a]nother or [d]one?: ')
            if action == 'a':
                return outcomes.CAPTURE_ITEM_NEXT
            elif action == 'd':
                self._bag.close()
                return outcomes.CAPTURE_ITEM_DONE
            else:
                action = None
