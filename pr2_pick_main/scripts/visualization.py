"""Visualization utilties for the state machine.
"""

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
import math
import random
import rospy
import tf

def publish_shelf(publisher, pose_stamped):
    """Publishes a shelf marker at a given pose.

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


def publish_order_bin(publisher):
    """Publishes the order bin marker based on tf.

    The pose is assumed to represent the bottom center of the order bin, with the
    +x direction pointing along the long axis of the bin and +z pointing up.

    Args:
      publisher: A visualization_msgs/Marker publisher
    """
    marker = Marker()
    marker.header.frame_id = 'order_bin'
    marker.header.stamp = rospy.Time().now()
    marker.ns = 'order_bin'
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.color.a = 1
    marker.color.r = 1
    marker.scale.x = 24 * 0.0254
    marker.scale.y = 14.5 * 0.0254
    marker.scale.z = 8 * 0.0254
    marker.pose.orientation.w = 1
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

def _get_pose_from_transform(transform):
    """Returns pose for transformation matrix.
    Args:
        transform (Matrix3x3): (I think this is the correct type.
            See ActionStepMarker as a reference for how to use.)
    Returns:
        Pose
    """
    pos = transform[:3, 3].copy()
    rot = tf.transformations.quaternion_from_matrix(transform)
    return Pose(
        Point(pos[0], pos[1], pos[2]),
        Quaternion(rot[0], rot[1], rot[2], rot[3])
    )


def publish_gripper(server, pose_stamped, name):
    """Publishes a marker representing a gripper.

    Code taken from action_step_marker.py in PR2/PbD.

    Args:
      server: An InteractiveMarkerServer
      pose_stamped: A PoseStamped giving the wrist_roll_link pose.
      name: string, a unique name for this gripper.
    """
    # Set angle of meshes based on gripper open vs closed.
    angle = 28 * math.pi / 180.0 # Fully open.
    STR_MESH_GRIPPER_FOLDER = 'package://pr2_description/meshes/gripper_v0/'
    STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + 'gripper_palm.dae'
    STR_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + 'l_finger.dae'
    STR_GRIPPER_FINGERTIP_FILE = STR_MESH_GRIPPER_FOLDER + 'l_finger_tip.dae'

    # Make transforms in preparation for meshes 1, 2, and 3.
    # NOTE(mbforbes): There are some magic numbers in here that are
    # used a couple times. Seems like a good candidate for
    # refactoring to constants, but I think they're more clear if
    # left in here as (a) they likely won't be changed, and (b) it's
    # easier to understand the computations with them here.
    transform1 = tf.transformations.euler_matrix(0, 0, angle)
    transform1[:3, 3] = [0.07691, 0.01, 0]
    transform2 = tf.transformations.euler_matrix(0, 0, -angle)
    transform2[:3, 3] = [0.09137, 0.00495, 0]
    t_proximal = transform1
    t_distal = tf.transformations.concatenate_matrices(
        transform1, transform2)

    # Create mesh 1 (palm).
    mesh1 = Marker()
    mesh1.header.frame_id = pose_stamped.header.frame_id
    mesh1.mesh_use_embedded_materials = True
    mesh1.type = Marker.MESH_RESOURCE
    mesh1.scale.x = 1.0
    mesh1.scale.y = 1.0
    mesh1.scale.z = 1.0
    mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
    mesh1.pose = pose_stamped.pose

    # Create mesh 2 (finger).
    mesh2 = Marker()
    mesh2.mesh_use_embedded_materials = True
    mesh2.type = Marker.MESH_RESOURCE
    mesh2.scale.x = 1.0
    mesh2.scale.y = 1.0
    mesh2.scale.z = 1.0
    mesh2.mesh_resource = STR_GRIPPER_FINGER_FILE
    mesh2.pose = _get_pose_from_transform(t_proximal)

    # Create mesh 3 (fingertip).
    mesh3 = Marker()
    mesh3.mesh_use_embedded_materials = True
    mesh3.type = Marker.MESH_RESOURCE
    mesh3.scale.x = 1.0
    mesh3.scale.y = 1.0
    mesh3.scale.z = 1.0
    mesh3.mesh_resource = STR_GRIPPER_FINGERTIP_FILE
    mesh3.pose = _get_pose_from_transform(t_distal)

    # Make transforms in preparation for meshes 4 and 5.
    quat = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_from_euler(math.pi, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, angle)
    )
    transform1 = tf.transformations.quaternion_matrix(quat)
    transform1[:3, 3] = [0.07691, -0.01, 0]
    transform2 = tf.transformations.euler_matrix(0, 0, -angle)
    transform2[:3, 3] = [0.09137, 0.00495, 0]
    t_proximal = transform1
    t_distal = tf.transformations.concatenate_matrices(
        transform1, transform2)

    # Create mesh 4 (other finger).
    mesh4 = Marker()
    mesh4.mesh_use_embedded_materials = True
    mesh4.type = Marker.MESH_RESOURCE
    mesh4.scale.x = 1.0
    mesh4.scale.y = 1.0
    mesh4.scale.z = 1.0
    mesh4.mesh_resource = STR_GRIPPER_FINGER_FILE
    mesh4.pose = _get_pose_from_transform(t_proximal)

    # Create mesh 5 (other fingertip).
    mesh5 = Marker()
    mesh5.mesh_use_embedded_materials = True
    mesh5.type = Marker.MESH_RESOURCE
    mesh5.scale.x = 1.0
    mesh5.scale.y = 1.0
    mesh5.scale.z = 1.0
    mesh5.mesh_resource = STR_GRIPPER_FINGERTIP_FILE
    mesh5.pose = _get_pose_from_transform(t_distal)

    # Append all meshes we made.
    control = InteractiveMarkerControl()
    control.markers = [mesh1, mesh2, mesh3, mesh4, mesh5]
    control.interaction_mode = InteractiveMarkerControl.NONE
    interactive_marker = InteractiveMarker()
    interactive_marker.controls = [control]
    interactive_marker.header.frame_id = pose_stamped.header.frame_id
    interactive_marker.pose = pose_stamped.pose
    interactive_marker.name = name

    server.insert(interactive_marker)
    server.applyChanges()

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
