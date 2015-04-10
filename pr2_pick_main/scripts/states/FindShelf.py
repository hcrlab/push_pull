from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import outcomes
import rospy
import smach
import tf


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

        shelf_base = PoseStamped()
        shelf_base.header.frame_id = 'base_footprint'
        shelf_base.pose.position.x = 2.2765
        shelf_base.pose.position.y = -0.53
        shelf_base.pose.position.z = 0
        shelf_base.pose.orientation.w = 1
        shelf_base.pose.orientation.x = 0
        shelf_base.pose.orientation.y = 0
        shelf_base.pose.orientation.z = 0

        shelf_odom = self._tf_listener.transformPose('odom_combined', shelf_base)

        transform = TransformStamped()
        transform.header.frame_id = 'odom_combined'
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf_odom.pose.position
        transform.transform.rotation = shelf_odom.pose.orientation
        transform.child_frame_id = 'shelf'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        self._tf_set = True
        return outcomes.FIND_SHELF_SUCCESS
