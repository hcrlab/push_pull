#! /usr/bin/python
import outcomes
import rospy
import smach
from states.MoveToBin import MoveToBin

class MockUserdata(object):
    ''' Stand-in for smach's userdata object '''

if __name__ == '__main__':
    rospy.init_node('MoveToBinTest')
    mock_userdata = MockUserdata
    mock_userdata.bin_id = 'A'
    state = MoveToBin()
    state.execute(mock_userdata)
