#! /usr/bin/python

import rospy
import smach
import sys
from visualization_msgs.msg import Marker

import outcomes
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, MoveHead, MoveTorso
from states.MoveToBin import MoveToBin
from std_msgs.msg import String

# Simple test to execute the MoveToBin state for a hard coded bin
# How to run:
# > rosrun pr2_pick_manipulation torso_service_node
# > rosrun pr2_pick_manipulation driver_service_node
# > rosrun pr2_pick_main mock_shelf_position.py
# > rosrun pr2_pick_main MoveToBinTest.py


class MockUserdata(object):
    ''' Stand-in for smach's userdata object '''

if __name__ == '__main__':
    rospy.init_node('MoveToBinTest')
    mock_userdata = MockUserdata
    mock_userdata.bin_id = 'G'
    if len(sys.argv) > 1:
        mock_userdata.bin_id = sys.argv[1]

    # get transform from base_link to shelf and stuff it into userdata
    # rospy.loginfo('Waiting for tf...')
    # listener = tf.TransformListener()
    # listener.waitForTransform(
    #     '/base_link',
    #     '/robot_shelf_position',
    #     rospy.Time(0),
    #     rospy.Duration(5)
    # )
    # (translation, rotation) = listener.lookupTransform(
    #     '/base_link',
    #     '/robot_shelf_position',
    #     rospy.Time(0)
    # )
    # mock_userdata.base_to_shelf_tf = (translation, rotation)

    tts = rospy.Publisher('/festival_tts', String)
    drive_angular = rospy.ServiceProxy('drive_angular_service', DriveAngular)
    drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
    move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
    move_head = rospy.ServiceProxy('move_head_service', MoveHead)
    markers = rospy.Publisher('pr2_pick_visualization', Marker)

    state = MoveToBin(tts, drive_linear, drive_angular, move_head, move_torso, markers)
    state.execute(mock_userdata)
