#!/usr/bin/env python

'''
A node providing inventory management services, e.g. what items are in what
bins. On startup, it takes in a filename specifying a file that contains
the contest json.
'''

import argparse
import copy
import json
import sys

from pr2_pick_contest.srv import GetItems, GetItemsResponse, GetTargetItems, \
    GetTargetItemsResponse, SetItems
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
            request.bin is a capital letter A-L
        @return a list of items
        '''
        bin_name = 'bin_' + request.bin
        print '{} contains {}'.format(bin_name, self.bin_contents[bin_name])
        return GetItemsResponse(self.bin_contents[bin_name])

    def get_target_items(self, request):
        '''
        Get a list of target items which originated in the specified bin
        @param request - a ros GetItems request
            request.bin is a capital letter A-L
        @return a list of items
        '''
        bin_name = 'bin_' + request.bin
        print 'target items of {} are {}'.format(
            bin_name,
            self.target_items[bin_name]
        )
        return GetTargetItemsResponse(self.target_items[bin_name])

    def set_items(self, request):
        '''
        Set the list of items currently believed to be in the specified bin
        @param request - a ros SetItems request
            request.bin is a capital letter A-L
            request.items is a list of items
        '''
        bin_name = 'bin_' + request.bin
        print 'changing contents of {} from {} to {}'.format(
            bin_name,
            self.bin_contents[bin_name],
            request.items,
        )
        self.bin_contents[bin_name] = request.items
        return True


if __name__ == "__main__":
    rospy.init_node('inventory')

    # get the filename from the command line
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'filename', metavar='FILE', type=str,
        help='json file specifying bin contents'
    )
    args = parser.parse_args(args=rospy.myargv()[1:])

    # create inventory from file
    inventory = Inventory(json.load(open(args.filename, 'r')))

    # start node and services
    rospy.Service('get_items', GetItems, inventory.get_items)
    rospy.Service('get_target_items', GetTargetItems, inventory.get_target_items)
    rospy.Service('set_items', SetItems, inventory.set_items)

    rospy.spin()
