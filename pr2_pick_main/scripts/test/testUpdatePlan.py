#! /usr/bin/env python
from bin_data import BinData
from states import UpdatePlan
import mock
import unittest


class TestUpdatePlan(unittest.TestCase):
    def setUp(self):
        tts = mock.Mock()
        tts.publish = mock.Mock()

        kwargs = {
            'tts': tts,
            'get_items': mock.Mock(),
            'set_items': mock.Mock(),
            'get_target_items': mock.Mock()
        }
        self.state = UpdatePlan(**kwargs)

    def test_first_bin_is_j(self):
        """Example test that checks if the first state returned is J.

        This test should be replaced with a real test once we pass the contest
        data through the state machine.
        """

        def get_items(bin_id):
            if bin_id == 'J':
                response = mock.Mock()
                response.items = ['oreo_mega_stuf']
                return response
            else:
                return []

        def get_target_items(bin_id):
            if bin_id == 'J':
                response = mock.Mock()
                response.items = ['oreo_mega_stuf']
                return response
            else:
                return []

        self.state._get_items = mock.Mock(side_effect=get_items)
        self.state._set_items = mock.Mock(return_value=True)
        self.state._get_target_items = mock.Mock(side_effect=get_target_items)

        user_data = mock.Mock()
        user_data.bin_data = {}
        for bin_id in 'ABCDEFGHIJKL':
            user_data.bin_data[bin_id] = BinData(bin_id, False, False, 3)

        outcome = self.state.execute(user_data)
        self.assertEqual(user_data.next_bin, 'J')
        self.assertEqual(user_data.output_bin_data['J'].visited, True)


if __name__ == '__main__':
    unittest.main()
