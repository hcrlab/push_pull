#!/usr/bin/env python

import argparse
import copy
import json
import sys

from pr2_pick_contest.srv import GetItems, GetItemsResponse, SetItems
import rospy


class Inventory(object):
    def __init__(self, contest_json_dict):
        '''
        @param contest_json_dict is a python dict from a json file in the
        format of this example: http://amazonpickingchallenge.org/example.json
        '''

        # this transformation is valid because "A single item in each bin
        # will be designated as a target item to be picked."
        # (http://amazonpickingchallenge.org/details.shtml)
        self.target_items = {
            order['bin']: [order['item']]
            for order in contest_json_dict['work_order']
        }

        self.bin_contents = copy.deepcopy(contest_json_dict['bin_contents'])

    def get_items(self, request):
        '''
        Get the items currently believed to be in the specified bin
        @param request - a ros GetItems request
            request.bin is a bin name in the form 'bin_*' where * is
                a capital letter A-L
        @return a list of items
        '''
        print self.bin_contents[request.bin]
        return GetItemsResponse(self.bin_contents[request.bin])

    def get_target_items(self, request):
        '''
        Get a list of target items which originated in the specified bin
        @param request - a ros GetItems request
            request.bin is a bin name in the form 'bin_*' where * is
                a capital letter A-L
        @return a list of items
        '''
        print self.target_items[request.bin]
        return GetItemsResponse(self.target_items[request.bin])

    def set_items(self, request):
        '''
        Set the list of items currently believed to be in the specified bin
        @param request - a ros SetItems request
            request.bin is a bin name in the form 'bin_*' where * is
                a capital letter A-L
            request.items is a list of items
        '''
        print 'changing contents of {} from {} to {}'.format(
            request.bin,
            self.bin_contents[request.bin],
            request.items,
        )
        self.bin_contents[request.bin] = request.items
        return True


if __name__ == "__main__":
    # get the filename from the command line
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'filename', metavar='FILE', type=str,
        help='json file specifying bin contents'
    )
    args = parser.parse_args()

    # create inventory from file
    inventory = Inventory(json.load(open(args.filename, 'r')))

    # start node and services
    rospy.init_node('inventory')
    rospy.Service('get_items', GetItems, inventory.get_items)
    rospy.Service('get_target_items', GetItems, inventory.get_target_items)
    rospy.Service('set_items', SetItems, inventory.set_items)

    rospy.spin()
