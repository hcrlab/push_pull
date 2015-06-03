#!/usr/bin/env python
"""Classifies objects based on descriptors.
"""

from pr2_pick_perception import ItemClassifier
from pr2_pick_perception import RosbagDataset
from pr2_pick_perception import TargetItemClassifier
from pr2_pick_perception.srv import ClassifyCluster
from pr2_pick_perception.srv import ClassifyTargetItem
import argparse
import rosbag
import rospy


def read_data(data_path):
    """Loads descriptor dataset from a rosbag file.
    """
    bag = rosbag.Bag(data_path)
    full_data = [(msg.descriptor, msg.label)
                 for topic, msg, time in bag.read_messages(topics=['examples'])
                 ]
    return full_data


if __name__ == '__main__':
    rospy.init_node('item_classifier')

    parser = argparse.ArgumentParser()
    parser.add_argument('data_file',
                        metavar='FILE',
                        type=str,
                        help='The saved rosbag dataset.')
    args = parser.parse_args(args=rospy.myargv()[1:])

    item_classifier = ItemClassifier(read_data(args.data_file))
    target_item_classifier = TargetItemClassifier(item_classifier)

    rospy.Service('item_classifier/classify_target_item', ClassifyTargetItem,
                  target_item_classifier.classify_request)
    rospy.Service('item_classifier/classify_cluster', ClassifyCluster,
                  item_classifier.classify_request)

    rospy.spin()
