#!/usr/bin/env python

from __future__ import division
from __future__ import print_function
from pr2_pick_perception.msg import ObjectMatchRequest
from pr2_pick_perception.msg import Cluster
from pr2_pick_perception.srv import MatchCluster, MatchClusterRequest
import collections
import os
import rosbag
import rospy
import sys

CONST_ITEM_NAMES = [
    "oreo_mega_stuf", "champion_copper_plus_spark_plug",
    "expo_dry_erase_board_eraser", "kong_duck_dog_toy",
    "genuine_joe_plastic_stir_sticks", "munchkin_white_hot_duck_bath_toy",
    "crayola_64_ct", "mommys_helper_outlet_plugs",
    "sharpie_accent_tank_style_highlighters",
    "kong_air_dog_squeakair_tennis_ball", "stanley_66_052",
    "safety_works_safety_glasses", "dr_browns_bottle_brush",
    "laugh_out_loud_joke_book", "cheezit_big_original",
    "paper_mate_12_count_mirado_black_warrior",
    "feline_greenies_dental_treats", "elmers_washable_no_run_school_glue",
    "mead_index_cards", "rolodex_jumbo_pencil_cup",
    "first_years_take_and_toss_straw_cup", "highland_6539_self_stick_notes",
    "mark_twain_huckleberry_finn", "kyjen_squeakin_eggs_plush_puppies",
    "kong_sitting_frog_dog_toy"
]


def read_data(data_path):
    bag = rosbag.Bag(data_path)
    for topic, msg, time in bag.read_messages(
        topics=['cell_pc', 'cropped_cloud']):
        if len(msg.labels) == 1:
            msg.cloud.header.frame_id = '/head_mount_kinect_rgb_optical_frame'
            yield (msg.cloud, msg.labels[0])


def read_data_dir(data_dir):
    data_files = os.listdir(data_dir)
    for data_file in data_files:
        data_path = os.path.join(data_dir, data_file)
        for example in read_data(data_path):
            yield example


def choose_1_item(items, exclude):
    """Yields each item in items, excluding some element."""
    for i in range(len(items)):
        if items[i] == exclude:
            continue
        yield [items[i]]


def choose_2_items(items, exclude):
    """Yields all unique subsets of size 2, except for the excluded item."""
    for i in range(len(items)):
        if items[i] == exclude:
            continue
        for j in range(i + 1, len(items)):
            if items[j] == exclude:
                continue
            yield [items[i], items[j]]


def run_experiment(test_dir, match_cluster, num_items):
    results = []
    data = read_data_dir(test_dir)
    num_correct = 0
    total = 0
    for cluster, label in data:
        other_item_generator = None
        if num_items == 2:
            other_item_generator = choose_1_item
        else:
            other_item_generator = choose_2_items

        obj_match_req = ObjectMatchRequest()
        cluster_msg = Cluster()
        cluster_msg.pointcloud = cluster
        cluster_msg.header.frame_id = cluster.header.frame_id
        obj_match_req.cluster = cluster_msg
        obj_match_req.descriptor_type = 'ORB'

        for other_items in other_item_generator(CONST_ITEM_NAMES, label):
            item_list = [label] + other_items
            obj_match_req.object_ids = item_list
            match_request = MatchClusterRequest()
            match_request.match = obj_match_req
            match_response = match_cluster(match_request)
            predicted_label = match_response.object_id
            if label == predicted_label:
                num_correct += 1
            else:
                print('{}, {}'.format(label, predicted_label))
            total += 1
            print(num_correct/total)
            results.append([label, predicted_label])
    print_accuracy(results, num_items)


def print_accuracy(results, items_in_bin):
    """Computes and prints the accuracy results.

    results: The results from run_experiment. A list of (item histogram, item
      name, predicted item name, confidence, list of other item names) tuples.
    items_in_bin: The number of items in the bin for these results.
    """
    num_correct = 0
    num_correct_by_item = collections.Counter()
    total_by_item = collections.Counter()
    for label, predicted in results:
        if label == predicted:
            num_correct_by_item[label] += 1
            num_correct += 1
        total_by_item[label] += 1
    print('Accuracy ({} items per bin): {}'.format(items_in_bin,
                                                   num_correct / len(results)))
    for item, num_correct in num_correct_by_item.most_common():
        print('  {}\t{}'.format(item, num_correct / total_by_item[item]))


if __name__ == '__main__':
    rospy.init_node('evaluate_obj_recognition')
    match_cluster = rospy.ServiceProxy('perception/match_cluster',
                                       MatchCluster,
                                       persistent=True)
    match_cluster.wait_for_service()
    test_dir = sys.argv[1]
    run_experiment(test_dir, match_cluster, 2)
