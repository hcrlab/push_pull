from geometry_msgs.msg import PoseStamped, Transform, TransformStamped, \
    Quaternion, Vector3
import math
import rospy
import smach
from std_msgs.msg import Header
import tf
from visualization_msgs.msg import Marker

import outcomes
from pr2_pick_perception.msg import ObjectDetectionRequest, ROI2d
from pr2_pick_perception.srv import LocalizeShelfRequest


class FindShelf(smach.State):
    '''Localizes the shelf.
    '''
    name = 'FIND_SHELF'

    def __init__(self, tts, localize_object, set_static_tf, markers, tf_listener,
                 **kwargs):
        '''Constructor for this state.

        Args:
          localize_object: The shelf localization service.
          set_static_tf: The service for setting static tfs.
        '''
        smach.State.__init__(
            self,
            outcomes=[outcomes.FIND_SHELF_SUCCESS, outcomes.FIND_SHELF_FAILURE],
            input_keys=['debug'])
        self._localize_object = localize_object
        self._set_static_tf = set_static_tf
        self._tf_listener = tf_listener
        self._tf_set = False
        self._tts = tts
        self._markers = markers

    def localize_shelf(self):
        '''Calls the object localization service to get the shelf position.

        If the service fails, or it returns a result outside of acceptable
        bounds, then it will try calling the service again, up to a total of 3
        tries.

        Returns: (success, pose), where success is whether or not we got a
        reasonable pose from the service, and pose is a PoseStamped message
        with the shelf's pose in the odom_combined frame.
        '''
        success = False
        shelf_ps = PoseStamped()  # The shelf pose returned by the service.
        shelf_odom = PoseStamped()  # Shelf pose in odom_combined frame.
        for try_num in range(5):
            self._localize_object.wait_for_service()
            obj_request = ObjectDetectionRequest()
            obj_request.obj_type = 'shelf'
            obj_request.region2d = ROI2d(top_left_x=-1,
                                         top_left_y=-1,
                                         bottom_right_x=-1,
                                         bottom_right_y=-1)
            request = LocalizeShelfRequest()
            request.object = obj_request
            response = self._localize_object(request)
            if len(response.locations.objects) == 0:
                rospy.logwarn('[FindShelf]: Shelf service returned no results.')
                continue
            shelf = response.locations.objects[0]
            rospy.loginfo('Shelf pose: {}'.format(shelf))
            shelf_ps.pose = shelf.pose
            shelf_ps.header = shelf.header

            try:
                shelf_odom = self._tf_listener.transformPose('odom_combined',
                                                             shelf_ps)
            except:
                rospy.logerr(
                    'No transform between {} and {} in FindShelf'.format(
                        shelf.header.frame_id, 'odom_combined'))
                continue

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [shelf_odom.pose.orientation.x, shelf_odom.pose.orientation.y,
                 shelf_odom.pose.orientation.z, shelf_odom.pose.orientation.w])
            rospy.loginfo('roll: {}, pitch: {}, yaw: {}'.format(
                180*roll/math.pi, 180*pitch/math.pi, 180*yaw/math.pi))

            # Check that the response is reasonable.
            if shelf_odom.pose.position.z < -0.1 or shelf_odom.pose.position.z > 0.1:
                rospy.logwarn('[FindShelf]: Shelf not on the ground.')
                continue

            success = True
            break

        if not success:
            return False, None
        return success, shelf_odom

    def execute(self, userdata):
        if (self._tf_set):
            return outcomes.FIND_SHELF_SUCCESS

        rospy.loginfo('Finding shelf.')
        self._tts.publish('Finding shelf.')

        success, shelf_odom = self.localize_shelf()
        if not success:
            rospy.logerr('[FindShelf]: Failed to localize shelf.')
            return outcomes.FIND_SHELF_FAILURE

        # Project onto the floor.
        # Adjusting pitch and roll to be 0 seems to make the model worse than just
        # letting it be slightly tilted.
        shelf_odom.pose.position.z = 0
        #_, _, yaw = tf.transformations.euler_from_quaternion(
        #    [shelf_odom.pose.orientation.x, shelf_odom.pose.orientation.y,
        #     shelf_odom.pose.orientation.z, shelf_odom.pose.orientation.w])
        #quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #shelf_odom.pose.orientation.x = quat[0]
        #shelf_odom.pose.orientation.y = quat[1]
        #shelf_odom.pose.orientation.z = quat[2]
        #shelf_odom.pose.orientation.w = quat[3]

        self._tts.publish('Found shelf.')

        # Possibly hard code the position of the shelf
        #shelf_base = PoseStamped()
        #shelf_base.header.frame_id = 'base_footprint'
        #shelf_base.pose.position.x = 2.27
        #shelf_base.pose.position.y = -0.61
        #shelf_base.pose.position.z = 0
        #shelf_base.pose.orientation.w = 1
        #shelf_base.pose.orientation.x = 0
        #shelf_base.pose.orientation.y = 0
        #shelf_base.pose.orientation.z = 0
        #shelf_odom = self._tf_listener.transformPose('odom_combined', shelf_base)

        # Publish static transform.
        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        self._tf_set = True

        # Publish marker
        marker = Marker()
        marker.header.frame_id = 'odom_combined'
        marker.header.stamp = rospy.Time().now()
        marker.ns = 'shelf'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = 'package://pr2_pick_perception/models/shelf/shelf.ply'
        marker.mesh_use_embedded_materials = True
        marker.action = Marker.ADD
        marker.pose = shelf_odom.pose
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.lifetime = rospy.Duration()

        # Need to wait for rviz for some reason.
        rate = rospy.Rate(1)
        while self._markers.get_num_connections() == 0:
            rate.sleep()
        self._markers.publish(marker)

        # Set up static a transform for each bin relative to shelf.
        # Bin origin is the front center of the bin opening, equidistant
        # from top edge and bottom edge of bin.
        shelf_depth = 0.87

        top_row_z = 1.524
        second_row_z = 1.308
        third_row_z = 1.073
        bottom_row_z = .806

        left_column_y = .2921
        center_column_y = 0.0
        right_column_y = -.2921

        bin_translations = {
            'A': Vector3(x=-shelf_depth/2., y=left_column_y, z=top_row_z),
            'B': Vector3(x=-shelf_depth/2., y=center_column_y, z=top_row_z),
            'C': Vector3(x=-shelf_depth/2., y=right_column_y, z=top_row_z),
            'D': Vector3(x=-shelf_depth/2., y=left_column_y, z=second_row_z),
            'E': Vector3(x=-shelf_depth/2., y=center_column_y, z=second_row_z),
            'F': Vector3(x=-shelf_depth/2., y=right_column_y, z=second_row_z),
            'G': Vector3(x=-shelf_depth/2., y=left_column_y, z=third_row_z),
            'H': Vector3(x=-shelf_depth/2., y=center_column_y, z=third_row_z),
            'I': Vector3(x=-shelf_depth/2., y=right_column_y, z=third_row_z),
            'J': Vector3(x=-shelf_depth/2., y=left_column_y, z=bottom_row_z),
            'K': Vector3(x=-shelf_depth/2., y=center_column_y, z=bottom_row_z),
            'L': Vector3(x=-shelf_depth/2., y=right_column_y, z=bottom_row_z),
        }

        for (bin_id, translation) in bin_translations.items():
            transform = TransformStamped(
                header=Header(
                    frame_id='shelf',
                    stamp=rospy.Time.now(),
                ),
                transform=Transform(
                    translation=translation,
                    rotation=Quaternion(w=1, x=0, y=0, z=0),
                ),
                child_frame_id='bin_{}'.format(bin_id),
            )
            self._set_static_tf.wait_for_service()
            self._set_static_tf(transform)

        if userdata.debug:
            raw_input('(Debug) Press enter to continue: ')

        return outcomes.FIND_SHELF_SUCCESS
