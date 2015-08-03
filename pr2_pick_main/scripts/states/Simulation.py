import outcomes
import rospy
import smach
from pr2_pick_main import handle_service_exceptions
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from actionlib import SimpleActionClient
import tf
import math
from geometry_msgs.msg import Point,PointStamped, Pose, PoseStamped
import visualization as viz
from visualization import IdTable
from std_msgs.msg import Header
from PushAway import PushAway
from PullForward import PullForward
from PushSideways import PushSideways
from TopSideways import TopSideways

class Simulation(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'SIMULATION'

    def __init__(self, tts, tuck_arms, move_head, tf_listener, **services):
        """Constructor for this state.

        Args:
          tts: A publisher for the TTS node
          tuck_arms: The tuck arms service proxy.
          move_head: The head service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.SIMULATION_SUCCESS, outcomes.SIMULATION_FAILURE],
            input_keys=['debug'],
            output_keys=['bounding_box'])

        self._markers = services['markers']
        self._tf_listener = tf.TransformListener()
        self.services = services

    def _publish(self, marker):
        """Publishes a marker to the given publisher.

        We need to wait for rviz to subscribe. If there are no subscribers to the
        topic within 5 seconds, we give up.
        """

        rate = rospy.Rate(1)
        for i in range(5):
            if self._markers.get_num_connections() > 0:
                self._markers.publish(marker)
                return
            rate.sleep()
        rospy.logwarn(
            'No subscribers to the marker publisher, did not publish marker.')

    @handle_service_exceptions(outcomes.SIMULATION_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Starting Simulation State!")


        bag = rosbag.Bag('bagfiles/notes_pose1.bag')
        for topic, msg, t in bag.read_messages(topics=['record']):
            msg.marker_boundingbox.header.stamp = rospy.Time().now()
            msg.marker_pointcloud.header.stamp = rospy.Time().now()
            self._publish(msg.marker_pointcloud)
            self._publish(msg.marker_boundingbox)
         
        bag.close()
        userdata.bounding_box = msg.boundingbox

        return outcomes.SIMULATION_SUCCESS
