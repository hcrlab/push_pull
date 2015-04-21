"""Visualization utilties for the state machine.
"""

from visualization_msgs.msg import Marker
import random
import rospy

def publish_shelf(publisher, pose_stamped):
    """Publishes a shelf marker at a give pose.

    The pose is assumed to represent the bottom center of the shelf, with the
    +x direction pointing along the depth axis of the bins and +z pointing up.

    Args:
      publisher: A visualization_msgs/Marker publisher
      pose_stamped: A PoseStamped message with the location, orientation, and
        reference frame of the shelf.
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


def publish_base(publisher, x, y, frame_id):
    """Publishes a marker representing the robot's navigation goal.
    The x and y arguments specify the center of the target.

    Args:
      publisher: A visualization_msgs/Marker publisher
      x: The x position of the center of the target position.
      y: The y position of the center of the target position.
      frame_id: The coordinate frame in which to interpret the target position.
        It's assumed that the frame's +z axis is in the same direction as
        base_footprint's +z axis.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = 'target_location'
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.03 / 2
    marker.pose.orientation.w = 1
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.scale.x = 0.67
    marker.scale.y = 0.67
    marker.scale.z = 0.03
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    marker.lifetime = rospy.Duration()
    _publish(publisher, marker)


def publish_cluster(publisher, points, frame_id, namespace, cluster_id):
    """Publishes a marker representing a cluster.
    The x and y arguments specify the center of the target.

    Args:
      publisher: A visualization_msgs/Marker publisher
      points: A list of geometry_msgs/Point
      frame_id: The coordinate frame in which to interpret the points.
      namespace: string, a unique name for a group of clusters.
      cluster_id: int, a unique number for this cluster in the given namespace.
    """
    marker = Marker()
    # TODO(jstn): Once the point clouds have the correct frame_id,
    # use them here.
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = namespace
    marker.id = cluster_id
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.color.r = random.random()
    marker.color.g = random.random()
    marker.color.b = random.random()
    marker.color.a = 1
    marker.points = points
    marker.scale.x = 0.01
    marker.scale.y = 0.01
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
