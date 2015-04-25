#! /usr/bin/env python

from states.UpdatePlan import UpdatePlan

# To execute GraspTool state while developing it

class MockUserdata(object):
    ''' Stand-in for smach's userdata object '''

class TestUpdatePlan(unittest.TestCase):
    def setUp(self):
    	print "candy"

	    tts = rospy.Publisher('/festival_tts', String)
        tts.publish = mock.Mock(side_effect=self.side_effect('tts'))

        get_items = mock.Mock(return_value={'book', 'box'})
        set_items = mock.Mock(return_value=true)
        get_target_items = mock.Mock(return_value={'book'})
        self.kwargs = {
            'tts': tts,
            'get_items': get_items,
            'set_items': set_items,
            'get_target_items': get_target_items
        }

	    self.state = UpdatePlan(self.kwargs)
	    self.mock_user_data = MockUserdata()
	    self.mock_user_data.debug = True

	def test_one(self):
		result = self.state.execute(self.mock_user_data)
	    print result


if __name__ == '__main__':
    unittest.main()