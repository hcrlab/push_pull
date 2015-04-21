"""Visualization utilties for the state machine.
"""

from visualization_msgs.msg import Marker
import rospy

def publish_shelf(publisher, pose_stamped):
    """Publishes a shelf marker at a give pose.
    The pose is assumed to represent the bottom center of the shelf, with the
    +x direction pointing along the depth axis of the bins and +z pointing up.
    """
    marker = Marker()
    marker.header.frame_id = pose_stamped.header.frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = 'shelf'
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = 'package://pr2_pick_perception/models/shelf/shelf.ply'
    marker.mesh_use_embedded_materials = True
    marker.action = Marker.ADD
    marker.pose = pose_stamped.pose
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.lifetime = rospy.Duration()
    _publish(publisher, marker)

def _publish(publisher, marker):
    """Publishes a marker to the given publisher.

    We need to wait for rviz to subscribe. If there are no subscribers to the
    topic within 5 seconds, we give up.
    """
    rate = rospy.Rate(1)
    for i in range(5):
        if publisher.get_num_connections() > 0:
            publisher.publish(marker)
            return
        rate.sleep()
    rospy.logwarn('No subscribers to the marker publisher, did not publish marker.')
