from geometry_msgs.msg import Point
from pr2_pick_perception.msg import MultiItemCloud
from pr2_pick_perception.srv import SegmentItems, SegmentItemsRequest
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import sys
import visualization as viz

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

def debug_visualization(messages):
    for cloud, labels in messages[:]:
        # Publish cloud
        #cloud_pub = rospy.Publisher('cloud', PointCloud2)
        #for i in range(2):
        #    if cloud_pub.get_num_connections() == 0:
        #        rospy.logwarn('Waiting for subscriber to cloud')
        #        rospy.sleep(1)
        #    else:
        #        cloud_pub.publish(cloud)
        #        break

        clusters = try_segmentation(cloud, labels)    

        # Publish visualization
        markers = rospy.Publisher('pr2_pick_visualization', Marker)
        for i, cluster in enumerate(clusters):
            points = pc2.read_points(cluster.pointcloud, skip_nans=True)
            point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
            if len(point_list) == 0:
                print 'Cluster {} has 0 points, skipping'.format(i)
                continue
            viz.publish_cluster(markers, point_list, 'map', 'clusters', i)

        raw_input('Press enter to continue: ')


def yield_grid_search_params():
    for y_scale_factor in [0.01, 0.1, 0.5, 1, 2, 4, 6, 10, 20, 40]:
        for color_meta_scale_factor in [0.01, 0.1, 0.5, 1, 2, 4, 6, 10, 20, 40]:
            for point_color in [10]:
                for region_color in [20]:
                    for min_cluster_size in [200]:
                        yield y_scale_factor, color_meta_scale_factor, point_color, region_color, min_cluster_size

def grid_search(messages):
    for ysf, cmsf, pc, rc, mcs in yield_grid_search_params():
        rospy.set_param('y_scale_factor', ysf)
        rospy.set_param('color_meta_scale_factor', cmsf)
        rospy.set_param('point_color', pc)
        rospy.set_param('region_color', rc)
        rospy.set_param('min_cluster_size', mcs)
        good_clusters = 0
        for cloud, labels in messages[:]:
            clusters = try_segmentation(cloud, labels)

            num_nonzero = 0
            for i, cluster in enumerate(clusters):
                points = pc2.read_points(cluster.pointcloud, skip_nans=True)
                point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
                if len(point_list) > 0:
                    num_nonzero += 1

            if num_nonzero == 2:
                good_clusters += 1
        rospy.loginfo('ysf: {}, cmsf: {}, pc: {}, rc: {}, mcs: {}; num dual-clusterings: {}'.format(ysf, cmsf, pc, rc, mcs, good_clusters))


if __name__ == '__main__':
    rospy.init_node('try_clustering')
    bag = rosbag.Bag(sys.argv[1])
    messages = read_bag(bag)
    debug_visualization(messages)
    #grid_search(messages)
