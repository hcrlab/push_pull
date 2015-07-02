from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy
from std_msgs.msg import String
import smach
import tf
from visualization_msgs.msg import Marker
import outcomes

class MoveObject(smach.State):
	''' Repositions an item with a tool. '''
	name = 'MOVE_OBJECT'

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.MOVE_OBJECT_SUCCESS,
                outcomes.MOVE_OBJECT_FAILURE,
            ]
        )	