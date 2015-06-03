#!/usr/bin/env python
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from pr2_pick_perception.msg import Cluster
from pr2_pick_perception.msg import MultiItemCloud
from pr2_pick_perception.srv import GetItemDescriptor
from pr2_pick_perception.srv import ClassifyCluster
from pr2_pick_perception.srv import SegmentItems, SegmentItemsRequest
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import random
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import sys
#import visualization as viz

def read_bag(bag):
    messages = []
    for topic, msg, t in bag.read_messages(topics=['cell_pc', 'cropped_cloud']):
        msg.cloud.header.frame_id = 'map'
        messages.append((msg.cloud, msg.labels))
    return messages

def segment_cloud(segment_items, cloud, labels):
    request = SegmentItemsRequest()
    request.cloud = cloud
    request.items = labels
    response = segment_items(request)
    return response.clusters.clusters

def try_segmentation(cloud, labels):
    segment_items = rospy.ServiceProxy('segment_items', SegmentItems)
    segment_items.wait_for_service()
    clusters = segment_cloud(segment_items, cloud, labels)
    return clusters

def try_over_segmentation(cloud, labels):
    segment_items = rospy.ServiceProxy('over_segment_items', SegmentItems)
    segment_items.wait_for_service()
    clusters = segment_cloud(segment_items, cloud, labels)
    return clusters

def try_kmeans_segmentation(cloud, labels):
    segment_items = rospy.ServiceProxy('kmeans_segment_items', SegmentItems)
    segment_items.wait_for_service()
    clusters = segment_cloud(segment_items, cloud, labels)
    return clusters

def try_over_kmeans_segmentation(cloud, labels):
    segment_items = rospy.ServiceProxy('over_kmeans_segment_items', SegmentItems)
    segment_items.wait_for_service()
    clusters = segment_cloud(segment_items, cloud, labels)
    return clusters

def classify(cluster, labels):
    get_item_descriptor = rospy.ServiceProxy('perception/get_item_descriptor', GetItemDescriptor)
    get_item_descriptor.wait_for_service()
    descriptor = get_item_descriptor(cluster=cluster).descriptor
    classify_cluster = rospy.ServiceProxy('item_classifier/classify_cluster', ClassifyCluster)
    classify_cluster.wait_for_service()
    response = classify_cluster(descriptor, labels)
    return response.label, response.confidence

def visualize_pipeline(visualization, messages, pause_fn):
    for cloud, labels in messages[:]:
        visualization.clear()
        visualization.visualize_cloud(cloud)

        clusters = try_over_segmentation(cloud, labels)    
        for i, cluster in enumerate(clusters):
            visualization.visualize_cluster(cluster)
        pause_fn()

#        visualization.clear()
#        clusters = try_kmeans_segmentation(cloud, labels)    
#        for i, cluster in enumerate(clusters):
#            visualization.visualize_cluster(cluster)
#        raw_input('Press enter to continue: ')
#
#        visualization.clear()
#        clusters = try_over_kmeans_segmentation(cloud, labels)    
#        for i, cluster in enumerate(clusters):
#            visualization.visualize_cluster(cluster)
#        raw_input('Press enter to continue: ')

        visualization.clear()
        clusters = try_segmentation(cloud, labels)    
        for i, cluster in enumerate(clusters):
            label, confidence = classify(cluster, labels)
            text_label = '{} ({:.4f})'.format(label, confidence)
            visualization.visualize_cluster(cluster, label=text_label)
        pause_fn()

def debug_visualization(visualization, messages):
    pause_fn = lambda: raw_input('Press enter to continue: ')
    visualize_pipeline(visualization, messages, pause_fn)

def demo_visualization(visualization, messages):
    pause_fn = lambda: rospy.sleep(5.0)
    visualize_pipeline(visualization, messages, pause_fn)

class Visualization(object):
    def __init__(self, cloud_pub, marker_pub):
        self._cloud_pub = cloud_pub
        self._marker_pub = marker_pub
        self._current_markers = []

    def visualize_cloud(self, cloud):
        self._publish(self._cloud_pub, cloud)

    def visualize_cluster(self, cluster, label=None):
        points = pc2.read_points(cluster.pointcloud, skip_nans=True)
        point_list = [Point(x=x, y=y-0.3, z=z) for x, y, z, rgb in points]
        if len(point_list) == 0:
            rospy.logwarn('Point list was of size 0, skipping.')
            return

        marker_id = len(self._current_markers)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time().now()
        marker.ns = 'clusters'
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        marker.color.a = 0.5 + random.random()
        marker.points = point_list
        marker.scale.x = 0.002
        marker.scale.y = 0.002
        marker.lifetime = rospy.Duration()
        self.visualize_marker(marker)
    
        if label is not None:
            center = [0, 0, 0]
            for point in point_list:
                center[0] += point.x
                center[1] += point.y
                center[2] += point.z
            center[0] /= len(point_list)
            center[1] /= len(point_list)
            center[2] /= len(point_list)
    
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = rospy.Time().now()
            text_marker.ns = 'labels'
            text_marker.id = marker_id + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = center[1] - 0.05
            text_marker.pose.position.y = center[1]
            text_marker.pose.position.z = center[2]
            text_marker.color.r = 1
            text_marker.color.g = 1
            text_marker.color.b = 1
            text_marker.color.a = 1
            text_marker.scale.z = 0.05
            text_marker.text = label
            text_marker.lifetime = rospy.Duration()
    
            self.visualize_marker(text_marker)

    def visualize_marker(self, marker):
        self._publish(self._marker_pub, marker)
        self._current_markers.append((marker.ns, marker.id))

    def clear(self):
        for marker_ns, marker_id in self._current_markers:
            self.delete_marker(self._marker_pub, marker_ns, marker_id)
        self._current_markers = []

    def delete_marker(self, publisher, ns, i):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = ns;
        marker.id = i;
        marker.action = Marker.DELETE
        self._publish(publisher, marker)

    def _publish(self, publisher, msg):
        for i in range(2):
            if publisher.get_num_connections() == 0:
                rospy.logwarn('Waiting for subscriber.')
                rospy.sleep(1)
            else:
                publisher.publish(msg)
                break


#def yield_grid_search_params():
#    for y_scale_factor in [0.01, 0.1, 0.5, 1, 2, 4, 6, 10, 20, 40]:
#        for color_meta_scale_factor in [0.01, 0.1, 0.5, 1, 2, 4, 6, 10, 20, 40]:
#            for point_color in [10]:
#                for region_color in [20]:
#                    for min_cluster_size in [200]:
#                        yield y_scale_factor, color_meta_scale_factor, point_color, region_color, min_cluster_size
#
#def grid_search(messages):
#    for ysf, cmsf, pc, rc, mcs in yield_grid_search_params():
#        rospy.set_param('y_scale_factor', ysf)
#        rospy.set_param('color_meta_scale_factor', cmsf)
#        rospy.set_param('point_color', pc)
#        rospy.set_param('region_color', rc)
#        rospy.set_param('min_cluster_size', mcs)
#        good_clusters = 0
#        for cloud, labels in messages[:]:
#            clusters = try_segmentation(cloud, labels)
#
#            num_nonzero = 0
#            for i, cluster in enumerate(clusters):
#                points = pc2.read_points(cluster.pointcloud, skip_nans=True)
#                point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
#                if len(point_list) > 0:
#                    num_nonzero += 1
#
#            if num_nonzero == 2:
#                good_clusters += 1
#        rospy.loginfo('ysf: {}, cmsf: {}, pc: {}, rc: {}, mcs: {}; num dual-clusterings: {}'.format(ysf, cmsf, pc, rc, mcs, good_clusters))


if __name__ == '__main__':
    rospy.init_node('try_clustering')
    bag = rosbag.Bag(sys.argv[1])
    messages = read_bag(bag)

    cloud_pub = rospy.Publisher('cloud', PointCloud2)
    marker_pub = rospy.Publisher('pr2_pick_visualization', Marker)
    visualization = Visualization(cloud_pub, marker_pub)

    while True:
        demo_visualization(visualization, messages)
    #grid_search(messages)
