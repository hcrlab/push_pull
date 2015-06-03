#!/usr/bin/env python
import rospy
import unittest
from pr2_pick_contest.srv import GetItems
from pr2_pick_contest.srv import GetTargetItems
from pr2_pick_contest.srv import LookupItem
from pr2_pick_contest import PickingStrategy

if __name__ == '__main__':
    rospy.init_node('test_strategy')
    get_items = rospy.ServiceProxy('inventory/get_items', GetItems)
    get_target_items = rospy.ServiceProxy('inventory/get_target_items', GetTargetItems)
    lookup_item = rospy.ServiceProxy('item_database/lookup_item', LookupItem)
    strategy = PickingStrategy(get_items, get_target_items, lookup_item)
    plan = strategy.get_plan_row_by_row()
    plan = strategy.get_plan_by_expected_value()
    rospy.loginfo(plan)
    rospy.spin()
