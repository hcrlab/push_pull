'''Visualization utilties for the state machine.
'''

from __future__ import division

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, \
    Marker
import math
import random
import rospy
import tf


class IdTable(object):
    '''
    Manage unique ids for objects markers. This way, callers don't have
    to worry about picking a unique integer for their object id. They
    can identify it with a string instead, which is less likely to collide.
    '''
    marker_ids = {}
    idx = 10000

    @staticmethod
    def get_id(string):
        if string not in IdTable.marker_ids:
            IdTable.marker_ids[string] = IdTable.idx
            IdTable.idx += 1
        return IdTable.marker_ids[string]


class MarkerNamespaces(object):
    ''' No global variables '''
    shelf = 'shelf'
    order_bin = 'order_bin'
    target_location = 'target_location'
    bounding_box = 'bounding_box'
    pose = 'pose'


def publish_shelf(publisher, pose_stamped):
    '''Publishes a shelf marker at a given pose.

    The pose is assumed to represent the bottom center of the shelf, with the
    +x direction pointing along the depth axis of the bins and +z pointing up.

    Args:
      publisher: A visualization_msgs/Marker publisher
      pose_stamped: A PoseStamped message with the location, orientation, and
        reference frame of the shelf.
    '''
    marker = Marker()
    marker.header.frame_id = pose_stamped.header.frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = MarkerNamespaces.shelf
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = 'package://pr2_pick_perception/models/shelf/shelf.ply'
    marker.mesh_use_embedded_materials = True
    marker.action = Marker.ADD
    marker.pose = pose_stamped.pose
    marker.color.a = 0.5
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.lifetime = rospy.Duration()
    _publish(publisher, marker)


def publish_order_bin(publisher):
    '''Publishes the order bin marker based on tf.

    The pose is assumed to represent the bottom center of the order bin, with the
    +x direction pointing along the long axis of the bin and +z pointing up.

    Args:
      publisher: A visualization_msgs/Marker publisher
    '''
    marker = Marker()
    marker.header.frame_id = 'order_bin'
    marker.header.stamp = rospy.Time().now()
    marker.ns = MarkerNamespaces.order_bin
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


def publish_base(publisher, pose_stamped):
    '''Publishes a marker representing the robot's navigation goal.
    The x and y arguments specify the center of the target.

    Args:
      publisher: A visualization_msgs/Marker publisher
      x: The x position of the center of the target position.
      y: The y position of the center of the target position.
      frame_id: The coordinate frame in which to interpret the target position.
        It's assumed that the frame's +z axis is in the same direction as
        base_footprint's +z axis.
    '''
    marker = Marker()
    marker.header.frame_id = pose_stamped.header.frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = MarkerNamespaces.target_location
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = pose_stamped.pose.position.x
    marker.pose.position.y = pose_stamped.pose.position.y
    marker.pose.position.z = 0.03 / 2
    marker.pose.orientation = pose_stamped.pose.orientation
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
    '''Publishes a marker representing a cluster.
    The x and y arguments specify the center of the target.

    Args:
      publisher: A visualization_msgs/Marker publisher
      points: A list of geometry_msgs/Point
      frame_id: The coordinate frame in which to interpret the points.
      namespace: string, a unique name for a group of clusters.
      cluster_id: int, a unique number for this cluster in the given namespace.
    '''
    marker = Marker()
    # TODO(jstn): Once the point clouds have the correct frame_id,
    # use them here.
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = namespace
    marker.id = 2 * cluster_id
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.color.r = random.random()
    marker.color.g = random.random()
    marker.color.b = random.random()
    marker.color.a = 0.5
    marker.points = points
    marker.scale.x = 0.002
    marker.scale.y = 0.002
    marker.lifetime = rospy.Duration()

    center = [0, 0, 0]
    for point in points:
        center[0] += point.x
        center[1] += point.y
        center[2] += point.z
    center[0] /= len(points)
    center[1] /= len(points)
    center[2] /= len(points)

    text_marker = Marker()
    text_marker.header.frame_id = frame_id
    text_marker.header.stamp = rospy.Time().now()
    text_marker.ns = namespace
    text_marker.id = 2 * cluster_id + 1
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = center[0] - 0.1
    text_marker.pose.position.y = center[1]
    text_marker.pose.position.z = center[2]
    text_marker.color.r = 1
    text_marker.color.g = 1
    text_marker.color.b = 1
    text_marker.color.a = 1
    text_marker.scale.z = 0.05
    text_marker.text = '{}'.format(cluster_id)
    text_marker.lifetime = rospy.Duration()

    _publish(publisher, marker)
    _publish(publisher, text_marker)

    return marker
def publish_bounding_box(publisher, pose_stamped, x, y, z, r, g, b, a,
                         marker_id):
    '''Publishes a marker representing a bounding box.

    Args:
      publisher: A visualization_msgs/Marker publisher
      pose_stamped: pose of marker
      x, y, z: dimensions of bounding box
      r, g, b, a: colour information for marker
      marker_id: id # for marker
    '''
    marker = Marker()
    marker.header.frame_id = pose_stamped.header.frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = MarkerNamespaces.bounding_box
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position = pose_stamped.pose.position
    marker.pose.orientation = pose_stamped.pose.orientation
    marker.scale.x = x
    marker.scale.y = y
    marker.scale.z = z
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    marker.lifetime = rospy.Duration()
    _publish(publisher, marker)
    return marker
def delete_bounding_box(publisher, marker_id):
    marker = Marker()
    marker.ns = MarkerNamespaces.bounding_box
    marker.action = Marker.DELETE
    marker.id = marker_id
    _delete(publisher, marker)



def publish_point(publisher, frame_id, point, r, g, b, a, marker_id, text=None):
    ''' Publish a 1 cm cube at the specified point '''
    pose = PoseStamped(
        header=Header(
            stamp=rospy.Time(0),
            frame_id=frame_id,
        ),
        pose=Pose(
            position=Point(x=point.x, y=point.y, z=point.z),
            orientation=Quaternion(0.0, 0, 0, 0),
        ),
    )
    publish_bounding_box(publisher, pose, 0.01, 0.01, 0.01, r, g, b, a, marker_id)

    if text is not None:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time().now()
        marker.ns = MarkerNamespaces.bounding_box
        marker.id = marker_id*2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation = Quaternion(0.0, 0, 0, 0)
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.lifetime = rospy.Duration()
        marker.text = text
        _publish(publisher, marker)


def publish_pose(publisher, pose_stamped, r, g, b, a, marker_id):
    '''Publishes a marker representing a bounding box.

    Args:
      publisher: A visualization_msgs/Marker publisher
      pose_stamped: pose of marker
      x, y, z: dimensions of bounding box
      r, g, b, a: colour information for marker
      marker_id: id # for marker 
    '''
    marker = Marker()
    marker.header.frame_id = pose_stamped.header.frame_id
    marker.header.stamp = rospy.Time().now()
    marker.ns = MarkerNamespaces.pose
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose_stamped.pose
    marker.scale.x = 0.07
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    marker.lifetime = rospy.Duration()
    _publish(publisher, marker)


def _get_pose_from_transform(transform):
    '''Returns pose for transformation matrix.
    Args:
        transform (Matrix3x3): (I think this is the correct type.
            See ActionStepMarker as a reference for how to use.)
    Returns:
        Pose
    '''
    pos = transform[:3, 3].copy()
    rot = tf.transformations.quaternion_from_matrix(transform)
    return Pose(Point(pos[0], pos[1], pos[2]),
                Quaternion(rot[0], rot[1], rot[2], rot[3]))


def publish_gripper(server, pose_stamped, name):
    '''Publishes a marker representing a gripper.

    Code taken from action_step_marker.py in PR2/PbD.

    Args:
      server: An InteractiveMarkerServer
      pose_stamped: A PoseStamped giving the wrist_roll_link pose.
      name: string, a unique name for this gripper.
    '''
    # Set angle of meshes based on gripper open vs closed.
    angle = 28 * math.pi / 180.0  # Fully open.
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
    t_distal = tf.transformations.concatenate_matrices(transform1, transform2)

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
        tf.transformations.quaternion_from_euler(0, 0, angle))
    transform1 = tf.transformations.quaternion_matrix(quat)
    transform1[:3, 3] = [0.07691, -0.01, 0]
    transform2 = tf.transformations.euler_matrix(0, 0, -angle)
    transform2[:3, 3] = [0.09137, 0.00495, 0]
    t_proximal = transform1
    t_distal = tf.transformations.concatenate_matrices(transform1, transform2)

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
    '''Publishes a marker to the given publisher.

    We need to wait for rviz to subscribe. If there are no subscribers to the
    topic within 5 seconds, we give up.
    '''
    # Figure out which of the two kinds of publishers we are using
    # 'interactive_marker_server': InteractiveMarkerServer('pr2_pick_interactive_markers'),
    # 'markers': rospy.Publisher('pr2_pick_visualization', Marker),

    if type(publisher) == rospy.topics.Publisher:
        rate = rospy.Rate(1)
        for i in range(5):
            if publisher.get_num_connections() > 0:
                publisher.publish(marker)
                return
            rate.sleep()
        rospy.logwarn(
            'No subscribers to the marker publisher, did not publish marker.')
    else:
        # stuff the marker into an interactive marker
        control = InteractiveMarkerControl()
        control.markers = [marker]
        control.orientation_mode = InteractiveMarkerControl.INHERIT
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        interactive_marker = InteractiveMarker()
        interactive_marker.controls = [control]
        interactive_marker.header.frame_id = marker.header.frame_id
        interactive_marker.pose = marker.pose
        interactive_marker.name = '{}_{}'.format(marker.ns, marker.id)

        publisher.insert(interactive_marker)
        publisher.applyChanges()

        rospy.loginfo('Created interactive marker named {}'.format(
                interactive_marker.name))

def _delete(publisher, marker):
    if type(publisher) == rospy.topics.Publisher:
        rospy.logwarn('Deletion not implemented for non-interactive markers')
    else:
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = marker.header.frame_id
        interactive_marker.name = '{}_{}'.format(marker.ns, marker.id)
        # because of a bug in interactive markers, only markers in the pending
        # queue can be erased, so we call insert here to queue it
        # https://github.com/ros-visualization/rviz/issues/446
        publisher.insert(interactive_marker)  # do not delete this line
        publisher.erase(interactive_marker)
        publisher.applyChanges()

        rospy.loginfo('Erased interactive marker named {}'.format(
                interactive_marker.name))
