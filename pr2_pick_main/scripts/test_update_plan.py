#! /usr/bin/env python

from states.UpdatePlan import UpdatePlan
import unittest
import rospy
import ros
from std_msgs.msg import String
import mock

class MockUserdata(object):
    ''' Stand-in for smach's userdata object '''

class TestUpdatePlan(unittest.TestCase):
    def side_effect(self, name, return_value=True):
        def wrapped(*args, **kwargs):
            rospy.loginfo('Calling {}{}'.format(name, args))
            return return_value
        return wrapped

    def setUp(self):
        print 'candy'
        tts = rospy.Publisher('/festival_tts', String)
        tts.publish = mock.Mock(side_effect=self.side_effect('tts'))

        get_items = mock.Mock(return_value={'book', 'box'})
        set_items = mock.Mock(return_value=True)
        get_target_items = mock.Mock(return_value={'book'})

        # get_items = mock.Mock(side_effect=self.side_effect('get_items'))
        # set_items = mock.Mock(side_effect=self.side_effect('set_items'))
        # get_target_items = mock.Mock(side_effect=self.side_effect('get_target_items'))

        kwargs = {
            'tts': tts,
            'get_items': get_items,
            'set_items': set_items,
            'get_target_items': get_target_items
        }

        self.kwargs = kwargs

        self.state = UpdatePlan()
        self.mock_user_data = MockUserdata()
        self.mock_user_data.debug = True

    def test_one(self):
        self.assertEqual(1, 1)
        result = self.state.execute(self.mock_user_data)
        print result


if __name__ == '__main__':
    unittest.main()