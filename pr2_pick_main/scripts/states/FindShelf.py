from geometry_msgs.msg import Orientation, PoseStamped, TransformStamped
import rospy
import smach
from std_msgs.msg import Header
import tf

import outcomes


class FindShelf(smach.State):
    """Localizes the shelf.
    """
    name = 'FIND_SHELF'

    def __init__(self, tts, localize_shelf, set_static_tf):
        """Constructor for this state.

        Args:
          localize_shelf: The shelf localization service.
          set_static_tf: The service for setting static tfs.
        """
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.FIND_SHELF_SUCCESS,
                outcomes.FIND_SHELF_FAILURE
            ]
        )
        self._localize_shelf = localize_shelf
        self._set_static_tf = set_static_tf
        self._tf_listener = tf.TransformListener()
        self._tf_set = False
        self._tts = tts

    def execute(self, userdata):
        rospy.loginfo('Finding shelf.')
        self._tts.publish('Finding shelf.')
        # TODO(jstn): Localization is not very precise yet.
        #self._localize_shelf.wait_for_service()
        #response = self._localize_shelf()
        #if len(response.locations.objects) == 0:
        #    return outcomes.FIND_SHELF_FAILURE
        #shelf = response.locations.objects[0]
        #rospy.loginfo(shelf)
        #shelf_ps = PoseStamped()
        #shelf_ps.pose = shelf.pose
        #shelf_ps.header = shelf.header
        #
        #shelf_odom = None
        #try:
        #    shelf_odom = self._tf_listener.transformPose('odom_combined', shelf_ps)
        #except:
        #    rospy.logerr('No transform between {} and {} in FindShelf'.format(
        #        shelf.header.frame_id, 'odom_combined'))
        #    return outcomes.FIND_SHELF_FAILURE

        if (self._tf_set):
            return outcomes.FIND_SHELF_SUCCESS

        # Until perception works, we're hard coding the shelf's tf.
        # Position is ahead and a bit to the right of the odom frame origin.
        # Orientation is the same as the odom frame's orientation.

        shelf_orientation = Orientation(w=1, x=0, y=0, z=0)

        shelf_base = PoseStamped()
        shelf_base.header.frame_id = 'base_footprint'
        shelf_base.pose.position.x = 2.2765
        shelf_base.pose.position.y = -0.53
        shelf_base.pose.position.z = 0
        shelf_base.pose.orientation = shelf_orientation

        shelf_odom = self._tf_listener.transformPose('odom_combined', shelf_base)

        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)

        # Set up static a transform for each bin relative to shelf.
        # Bin origin is the front center of the bin opening.
        shelf_depth = 0.87

        top_row_z = 165.735
        second_row_z = 142.24
        third_row_z = 118.745
        bottom_row_z = 93.98

        left_column_y = 29.21
        center_column_y = 0.0
        right_column_y = -29.21

        # These could remain static once perception works or we could try
        # localizing each bin individually.
        bin_translations = {
            'A': Position(x=shelf_depth/2, y=left_column_y, z=top_row_z),
            'B': Position(x=shelf_depth/2, y=center_column_y, z=top_row_z),
            'C': Position(x=shelf_depth/2, y=right_column_y, z=top_row_z),
            'D': Position(x=shelf_depth/2, y=left_column_y, z=second_row_z),
            'E': Position(x=shelf_depth/2, y=center_column_y, z=second_row_z),
            'F': Position(x=shelf_depth/2, y=right_column_y, z=second_row_z),
            'G': Position(x=shelf_depth/2, y=left_column_y, z=third_row_z),
            'H': Position(x=shelf_depth/2, y=center_column_y, z=third_row_z),
            'I': Position(x=shelf_depth/2, y=right_column_y, z=third_row_z),
            'J': Position(x=shelf_depth/2, y=left_column_y, z=bottom_row_z),
            'K': Position(x=shelf_depth/2, y=center_column_y, z=bottom_row_z),
            'L': Position(x=shelf_depth/2, y=right_column_y, z=bottom_row_z),
        }

        for bin_id, translation in bin_translations:
            transform = TransformStamped(
                header=Header(
                    frame_id='shelf',
                    stamp=rospy.Time.now(),
                    ),
                transform=Transform(
                    translation=translation,
                    rotation=shelf_orientation,
                    )
                child_frame_id=bin_id,
                )
            self._set_static_tf.wait_for_service()
            self._set_static_tf(transform)

        self._tf_set = True
        return outcomes.FIND_SHELF_SUCCESS
