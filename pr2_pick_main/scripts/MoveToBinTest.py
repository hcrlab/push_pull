#! /usr/bin/python
import outcomes
import rospy
import smach
from states.MoveToBin import MoveToBin
import tf

# Simple test to execute the MoveToBin state for a hard coded bin
# How to run:
# > rosrun pr2_pick_manipulation torso_service_node
# > rosrun pr2_pick_manipulatn driver_service_node
# > rosrun pr2_pick_main mock_shelf_position.py
# > rosrun pr2_pick_main MoveToBinTest.py

class MockUserdata(object):
    ''' Stand-in for smach's userdata object '''

if __name__ == '__main__':
    rospy.init_node('MoveToBinTest')
    mock_userdata = MockUserdata
    mock_userdata.bin_id = 'G'

    # get transform from base_link to shelf and stuff it into userdata
    rospy.loginfo('Waiting for tf...')
    listener = tf.TransformListener()
    listener.waitForTransform(
        '/base_link',
        '/robot_shelf_position',
        rospy.Time(0),
        rospy.Duration(5)
    )
    (translation, rotation) = listener.lookupTransform(
        '/base_link',
        '/robot_shelf_position',
        rospy.Time(0)
    )
    mock_userdata.base_to_shelf_tf = (translation, rotation)

    state = MoveToBin()
    state.execute(mock_userdata)
