#!/usr/bin/env python
"""Evaluates the accuracy of color histograms.

The dataset is given as a rosbag file with
pr2_pick_perception/DescriptorExample messages.

Usage: python evaluate_color_histograms.py descriptors_4.bag
"""

from __future__ import division
from __future__ import print_function
import collections
import json
import numpy as np
from sklearn.metrics import precision_recall_curve
from sklearn.cross_validation import train_test_split
import random
import rosbag
import sys
from pr2_pick_perception import ItemClassifier
from pr2_pick_perception import TargetItemClassifier
from pr2_pick_perception.srv import GetItemDescriptor, GetItemDescriptorRequest


def read_data(data_path, train_size):
    """Loads histogram dataset from a rosbag file.

    Returns two lists, both of type [(np.array, string)].
    The np.array is the histogram and the string is the label.
    """
    bag = rosbag.Bag(data_path)
    full_data = [(msg.descriptor, msg.label)
                 for topic, msg, time in bag.read_messages(topics=['examples'])
                 ]
    split_data = train_test_split(full_data, train_size=train_size)
    return split_data[0], split_data[1]


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


def run_experiment(classifier, training_set, test_set, items_in_bin):
    """Runs the item by item classifier experiment.

    For each item in the test set, we try all possible combinations of other
    items to put in the bin (items_in_bin is either 2 or 3).

    Running the experiment includes classifying all elements of the test set
    and printing out evaluation results.

    classifier: The classifier to evaluate.
    training_set: The training set, from read_data.
    test_set: The set set, from read_data.
    items_in_bin: The number of items in the bin (either 2 or 3).
    """
    results = []
    train_items = list(set([l for d, l in training_set]))
    for test_histogram, test_label in test_set:
        other_item_generator = None
        if items_in_bin == 2:
            other_item_generator = choose_1_item
        else:
            other_item_generator = choose_2_items

        for other_items in other_item_generator(train_items, test_label):
            item_list = [test_label] + other_items
            predicted, confidence = classifier.classify(test_histogram,
                                                        item_list)
            results.append((test_histogram, test_label, predicted, confidence,
                            other_items))
    print_accuracy(results, items_in_bin)
    #write_results_file(results, items_in_bin, distance_name, classifier)
    #write_precision_recall(results, items_in_bin, distance_name, classifier)


def run_target_experiment(target_item_classifier, test_set, items_in_bin):
    """Runs the target item experiment.

    For each item in the test set, we try all possible combinations of other
    items to put in the bin (items_in_bin is either 2 or 3). We then see if
    we can correctly identify the test item out of the 2 or 3 items.

    target_item_classifier: The TargetItemClassifier to evaluate.
    test_set: The test set, from read_data.
    items_in_bin: The number of items in the bin (either 2 or 3).
    """
    results = []

    other_item_generator = None
    if items_in_bin == 2:
        other_item_generator = choose_1_item
    else:
        other_item_generator = choose_2_items
    
    for test_histogram, test_label in test_set:
        for other_items in other_item_generator(test_set, (test_histogram, test_label)):
            other_histograms = [h for h, l in other_items]
            other_labels = [l for h, l in other_items]
            all_labels = [test_label] + other_labels
            predicted_ind, confidence = target_item_classifier.classify(
                [test_histogram] + other_histograms,
                test_label,
                all_labels)
            predicted = all_labels[predicted_ind]
            results.append((test_histogram, test_label, predicted,
                            confidence, other_items))
    print_accuracy(results, items_in_bin)


def print_accuracy(results, items_in_bin):
    """Computes and prints the accuracy results.

    results: The results from run_experiment. A list of (item histogram, item
      name, predicted item name, confidence, list of other item names) tuples.
    items_in_bin: The number of items in the bin for these results.
    """
    num_correct = 0
    num_correct_by_item = collections.Counter()
    total_by_item = collections.Counter()
    for histogram, item, predicted, confidence, other_items in results:
        if item == predicted:
            num_correct_by_item[item] += 1
            num_correct += 1
        total_by_item[item] += 1
    print('Accuracy ({} items per bin): {}'.format(items_in_bin,
                                                   num_correct / len(results)))
    for item, num_correct in num_correct_by_item.most_common():
        print('  {}\t{}'.format(item, num_correct / total_by_item[item]))

# TODO(jstn): Reinstate these if there's a need for more information.
#def write_results_file(results, items_in_bin, distance_name, classifier):
#    """Writes the results out to a .tsv file.
#
#    results: The results from run_experiment. A list of (item histogram, item
#      name, predicted item name, confidence, list of other item names) tuples.
#    items_in_bin: The number of items in the bin for these results.
#    distance_name: The distance metric for these results.
#    classifier: The classifier used for these results.
#    """
#    output_file = open('{}_{}.tsv'.format(items_in_bin, distance_name), 'w')
#    for histogram, item, predicted, confidence, other_items in results:
#        correct = 1 if item == predicted else 0
#        other_string = '\t'.join(other_items)
#
#        # Show extra columns if the classification was wrong.
#        detail = ''
#        if predicted != item:
#            predicted_histogram, actual_histogram, predicted_distance, actual_distance = classifier.get_details(
#                histogram, predicted, item)
#            detail = '\t' + '\t'.join(
#                [str(histogram), str(predicted_histogram),
#                 str(actual_histogram), str(predicted_distance),
#                 str(actual_distance)])
#        print('{}\t{}\t{}\t{}\t{}{}'.format(item, predicted, correct,
#                                            confidence, other_string, detail),
#              file=output_file)
#
#
#def write_precision_recall(results, items_in_bin, distance_name, classifier):
#    """Writes the precision-recall curve to a file.
#
#    results: The results from run_experiment. A list of (item histogram, item
#      name, predicted item name, confidence, list of other item names) tuples.
#    items_in_bin: The number of items in the bin for these results.
#    distance_name: The distance metric for these results.
#    classifier: The classifier used for these results.
#    """
#    output_file = open('prcurve_{}_{}.tsv'.format(items_in_bin, distance_name),
#                       'w')
#    corrects = [1 if actual == predicted else 0
#                for _, actual, predicted, _, _ in results]
#    confidences = [c for _, _, _, c, _ in results]
#    precisions, recalls, thresholds = precision_recall_curve(corrects,
#                                                             confidences)
#    for precision, recall, threshold in zip(precisions, recalls, thresholds):
#        print('{}\t{}\t{}'.format(precision, recall, threshold),
#              file=output_file)

if __name__ == '__main__':
    dataset_file = sys.argv[1]
    training_set, test_set = read_data(dataset_file, 0.9)

    classifier = ItemClassifier(training_set)
    target_item_classifier = TargetItemClassifier(classifier)

    print('Item by item classification')
    run_experiment(classifier, training_set, test_set, 2)
    run_experiment(classifier, training_set, test_set, 3)

    print('Target item classification')
    run_target_experiment(target_item_classifier, test_set, 2)
    run_target_experiment(target_item_classifier, test_set, 3)
