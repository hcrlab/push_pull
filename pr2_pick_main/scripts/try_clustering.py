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

if __name__ == '__main__':
    rospy.init_node('try_clustering')
    bag = rosbag.Bag(sys.argv[1])
    messages = read_bag(bag)

    cloud, labels = messages[1]

    # Publish cloud
    cloud_pub = rospy.Publisher('cloud', PointCloud2)
    while cloud_pub.get_num_connections() == 0:
        rospy.sleep(1)
    cloud_pub.publish(cloud)

    segment_items = rospy.ServiceProxy('segment_items', SegmentItems)
    segment_items.wait_for_service()
    clusters = segment_cloud(segment_items, cloud, labels)

    # Publish visualization
    markers = rospy.Publisher('pr2_pick_visualization', Marker)
    for i, cluster in enumerate(clusters):
        points = pc2.read_points(cluster.pointcloud, skip_nans=True)
        point_list = [Point(x=x, y=y, z=z) for x, y, z, rgb in points]
        viz.publish_cluster(markers, point_list, 'map', 'clusters', i)
