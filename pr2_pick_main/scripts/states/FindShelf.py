from pr2_pick_perception.msg import Object
from geometry_msgs.msg import TransformStamped
import outcomes
import rospy
import smach


class FindShelf(smach.State):
    """Localizes the shelf.
    """
    name = 'FIND_SHELF'

    def __init__(self, localize_shelf, set_static_tf):
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

    def execute(self, userdata):
        rospy.loginfo('Finding shelf.')
        self._localize_shelf.wait_for_service()
        response = self._localize_shelf()
        if len(response.objects) == 0:
            return outcomes.FIND_SHELF_FAILURE
        shelf = response.objects[0]
        transform = TransformStamped()
        transform.header = shelf.header
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = shelf.pose.position
        transform.transform.rotation = shelf.pose.orientation
        transform.child_frame_id = 'shelf'
        self._set_static_tf.wait_for_service()
        self._set_static_tf(transform)
        return outcomes.FIND_SHELF_SUCCESS
