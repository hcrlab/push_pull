#! /usr/bin/python

import rospy
import smach
import sys
import tf

import outcomes
from pr2_pick_manipulation.srv import DriveLinear, MoveTorso
from states.MoveToBin import MoveToBin

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

    move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
    drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
    state = MoveToBin(drive_linear, move_torso)
    state.execute(mock_userdata)
