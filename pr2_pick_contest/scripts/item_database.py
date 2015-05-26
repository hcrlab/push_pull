#!/usr/bin/env python
'''Provides information about contest items. '''
import argparse
import copy
import json
import sys

from pr2_pick_contest.msg import ItemModel
from pr2_pick_contest.srv import LookupItem, LookupItemResponse
from pr2_pick_perception.msg import ColorHistogram
import rospy


class ItemDatabase(object):
    def __init__(self, item_data):
        self._item_data = {}
        for item, data_dict in item_data.items():
            model = ItemModel()
            model.is_graspable = data_dict['graspable']
            model.allowed_grasps = data_dict['allowed_grasps']
            model.grasp_wide_end = data_dict['grasp_wide_end']
            model.grasp_multiple_heights = data_dict['grasp_multiple_heights']
            model.allow_finger_collisions = data_dict['allow_finger_collisions']
            model.zero_width_grasp = data_dict['zero_width_grasp']
            model.bonus_points = data_dict['bonus_points']
            model.grasp_effort = data_dict['grasp_effort']
            model.success_prior = data_dict['success_prior']
            model.speech_name = data_dict['speech_name']

            self._item_data[item] = model

    def lookup_item(self, request):
        response = LookupItemResponse()
        response.model = self._item_data[request.item]
        return response


if __name__ == '__main__':
    rospy.init_node('item_database')

    # get the filename from the command line
    parser = argparse.ArgumentParser()
    parser.add_argument('filename',
                        metavar='FILE',
                        type=str,
                        help='json file specifying item models')
    args = parser.parse_args(args=rospy.myargv()[1:])

    item_db = ItemDatabase(json.load(open(args.filename, 'r')))

    # start node and services
    rospy.Service('item_database/lookup_item', LookupItem, item_db.lookup_item)

    rospy.spin()
