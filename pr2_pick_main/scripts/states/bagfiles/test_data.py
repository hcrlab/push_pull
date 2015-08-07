import rosbag
import ros
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image

def _publish(publisher_markers, marker):
	"""Publishes a marker to the given publisher.
	We need to wait for rviz to subscribe. If there are no subscribers to the
	topic within 5 seconds, we give up.
	"""

	rate = rospy.Rate(1)
	for i in range(5):
	    if publisher_markers.get_num_connections() > 0:
	        publisher_markers.publish(marker)
	        return
	    rate.sleep()
	rospy.logwarn(
	    'No subscribers to the marker publisher, did not publish marker.')

def main():
	rospy.init_node('test_bag')
	publisher_markers = rospy.Publisher('pr2_pick_visualization', Marker)
	publisher_image = rospy.Publisher("/head_mount_kinect/rgb/image_color", Image)

	bag = rosbag.Bag('crayola/crayola_pose1_rotation1.bag')
	#print("Pose: " + str(i) + " Rotation: " + str(j) + "\n" )
	for topic, msg, t in bag.read_messages(topics=['record']):
		msg.marker_boundingbox.header.stamp = rospy.Time(0)
		msg.marker_pointcloud.header.stamp = rospy.Time(0)
		for i in range(1000):
			_publish(publisher_markers, msg.marker_pointcloud)
			_publish(publisher_markers, msg.marker_boundingbox)
			_publish(publisher_image, msg.image)
		print(msg.marker_boundingbox.header.frame_id)
		print(msg.is_graspable)
	#raw_input("Press enter for next pose/rotation")     
	bag.close()

main()