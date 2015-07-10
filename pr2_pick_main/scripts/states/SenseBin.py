from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception import DataSaver
from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import SegmentItemsRequest
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import outcomes
import rospy
import smach
from pr2_pick_manipulation.srv import MoveHead
import visualization as viz


class SenseBin(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_BIN'

    def __init__(self, tts, crop_shelf, markers,move_head, **kwargs):
        """Constructor for this state.

        tts: The text-to-speech publisher.
        crop_shelf: The shelf cropping service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.SENSE_BIN_SUCCESS, outcomes.SENSE_BIN_NO_OBJECTS,
                      outcomes.SENSE_BIN_FAILURE],
            input_keys=['bin_id', 'debug', 'current_target',
                        'current_bin_items', 're_sense_attempt', 'previous_item'],
            output_keys=['clusters', 'target_cluster', 'target_descriptor',
                         'target_model', 're_grasp_attempt', 'current_target'])
        self._tts = tts
        self._crop_shelf = crop_shelf
        self._segment_items = kwargs['segment_items']
        self._markers = markers
        self._get_item_descriptor = kwargs['get_item_descriptor']
        self._classify_target_item = kwargs['classify_target_item']
        self._lookup_item = kwargs['lookup_item']
	self._move_head = move_head

    @handle_service_exceptions(outcomes.SENSE_BIN_FAILURE)
    def execute(self, userdata):

        if 're_sense_attempt' in userdata and userdata.re_sense_attempt:
            userdata.re_grasp_attempt = True
        else:
            userdata.re_grasp_attempt = False

        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        self._tts.publish('Sensing bin {}'.format(userdata.bin_id))
	self._move_head.wait_for_service()
        move_head_success = self._move_head(0, 0, 0, 'bin_K')
  #       self.target_items = ["highland_6539_self_stick_notes", "crayola_64_ct"] 
  #       num_items = len(self.target_items)
  #       if( not userdata.previous_item ):
  #           userdata.current_target = self.target_items[0]
  #       else:
  #           count = 0 
  #           for item in self.target_items:
        # rospy.loginfo("item: " + item)
        # rospy.loginfo("previous item: " + userdata.previous_item)
  #               if(item == userdata.previous_item):
        #     rospy.loginfo("Count: " + str(count))
        #     rospy.loginfo("Num items: " + str(num_items))
  #                   if(count != num_items - 1):
  #                       userdata.current_target = self.target_items[count + 1]
  #                   else:
  #                       userdata.current_target = self.target_items[0]
        # count = count + 1

        
        self._lookup_item.wait_for_service()
        lookup_response = self._lookup_item(item=userdata.current_target)
        target_model = lookup_response.model
        userdata.target_model = target_model
        rospy.loginfo("Place item : " + userdata.current_target)
        self._tts.publish('Place item {}'.format(target_model.speech_name))
        rospy.sleep(2)
        raw_input("Press enter after placing the item.")
        current_bin_items = userdata.current_target

        # Crop shelf.
        crop_request = CropShelfRequest(cellID=userdata.bin_id)
        self._crop_shelf.wait_for_service()
        crop_response = self._crop_shelf(crop_request)

        # Save point cloud for later analysis
        #try:
        #    if len(current_bin_items) > 1:
        #        filename = '_'.join([x[:5] for x in [userdata.current_target] + current_bin_items])
        #        filename += '.bag'
        #        data_saver = DataSaver('/tmp/cell_pc', filename)
        #        data_saver.save_message('cell_pc', crop_response.cloud)
        #        data_saver.close()
        #        rospy.loginfo('Saved cropped point cloud to /tmp/cell_pc/{}'.format(filename))
        #except e:
        #    rospy.logerr('Failed to save point cloud data: {}'.format(e))

        # Segment items.
        segment_request = SegmentItemsRequest(cloud=crop_response.cloud, items=current_bin_items)
        self._segment_items.wait_for_service()
        segment_response = self._segment_items(segment_request)
        clusters = segment_response.clusters.clusters
        userdata.clusters = clusters
        rospy.loginfo('[SenseBin] Found {} clusters.'.format(
            len(clusters)))
        if len(clusters) == 0:
            rospy.logerr('[SenseBin]: No clusters found!')
            return outcomes.SENSE_BIN_FAILURE

        descriptors = []
        for i, cluster in enumerate(clusters):
            # Publish visualization
            points = pc2.read_points(cluster.pointcloud, skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
            if len(point_list) == 0:
                rospy.logwarn('[SenseBin]: Cluster with 0 points returned!')
                continue
            #viz.publish_cluster(self._markers, point_list,
            #                    'bin_{}'.format(userdata.bin_id),
            #                    'bin_{}_items'.format(userdata.bin_id), i)

            # Get descriptor
            self._get_item_descriptor.wait_for_service()
            response = self._get_item_descriptor(cluster=cluster)
            descriptors.append(response.descriptor)

        if len(current_bin_items) != len(descriptors):
            rospy.logwarn((
                '[SenseBin] Only {} descriptors from {} clusters returned, '
                'expected {} items in bin'
            ).format(len(descriptors), len(clusters),
                     len(current_bin_items)))

        # Classify which cluster is the target item.
        if len(descriptors) == 0:
            rospy.logerr('[SenseBin]: No descriptors found!')
            return outcomes.SENSE_BIN_FAILURE
        self._classify_target_item.wait_for_service()
        response = self._classify_target_item(
            descriptors=descriptors,
            target_item=userdata.current_target,
            all_items=current_bin_items)
        index = response.target_item_index
        userdata.target_cluster = clusters[index]
        userdata.target_descriptor = descriptors[index]
        rospy.loginfo(
            'Classified cluster #{} as target item ({} confidence)'.format(
                index, response.confidence))

        if userdata.debug:
            raw_input('[SenseBin] Press enter to continue: ')
        return outcomes.SENSE_BIN_SUCCESS
