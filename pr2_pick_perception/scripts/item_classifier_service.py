#!/usr/bin/env python
"""Classifies objects based on descriptors.
"""

from pr2_pick_perception import ItemClassifier
from pr2_pick_perception import RosbagDataset
from pr2_pick_perception import TargetItemClassifier
from pr2_pick_perception.srv import ClassifyTargetItem
import argparse

if __name__ == '__main__':
    rospy.init_node('item_classifier')

    parser = argparse.ArgumentParser()
    parser.add_argument('data_dir',
                        metavar='DIR',
                        type=str,
                        help='Directory with saved rosbag dataset.')
    args = parser.parse_args(args=rospy.myargv()[1:])

    rosbag_dataset = RosbagDataset(args.data_dir)
    item_classifier = ItemClassifier(rosbag_dataset)
    target_item_classifier = TargetItemClassifier(item_classifier)

    rospy.Service('item_classifier/classify_target_item', ClassifyTargetItem,
                  target_item_classifier.classify_request)

    rospy.spin()
