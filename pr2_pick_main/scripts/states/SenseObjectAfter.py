from pr2_pick_main import handle_service_exceptions
from pr2_pick_perception import DataSaver
from pr2_pick_perception.srv import CropShelfRequest
from pr2_pick_perception.srv import SegmentItemsRequest
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import outcomes
import rospy
import smach
from pr2_pick_manipulation.srv import MoveHead
import visualization as viz


class SenseObjectAfter(smach.State):
    name = 'SENSE_OBJECT_AFTER'

    ### TODO once sense before finalized ###