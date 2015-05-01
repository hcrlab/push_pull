#!/usr/bin/env python
"""Evaluates the accuracy of color histograms.

The training set is item_models.json, the test set is another JSON file (same
format as training set) with unseen color histograms for each item.

Usage: python evaluate_color_histograms.py item_models.json test_set.json
"""

from __future__ import division
from __future__ import print_function
import collections
import json
import numpy as np
from sklearn.metrics import precision_recall_curve
import sys


# Distance functions
def l2_distance(h1, h2):
    return np.linalg.norm(h1 - h2)


def l1_distance(h1, h2):
    return np.linalg.norm(h1 - h2, ord=1)


def cosine_distance(h1, h2):
    return h1.dot(h2) / (np.linalg.norm(h1) * np.linalg.norm(h2))


class ColorHistogramClassifier(object):
    def __init__(self, training_data, distance_metric):
        """Constructor.

        training_data: The training data, from read_data.
        distance_metric: One of the predefined distance functions.
        """
        self._data = training_data
        self._distance = distance_metric
        self._printed = 0  # Set to non-zero to debug classifications

    def classify(self, histogram, labels):
        """Returns the most likely label assignment for the given histogram,
        and a confidence for that label.

        histogram: The histogram to classify.
        labels: A list of item names to possibly classify the histogram as.
        """
        second_min_distance = None
        min_distance = None
        best_label = None
        if self._printed > 0:
            print('histogram: {}'.format(histogram))
        for label in labels:
            label_histograms = self._data[label]
            for label_histogram in label_histograms:
                distance = self._distance(histogram, label_histogram)
                if self._printed > 0:
                    print('  label: {}, histogram: {}, distance: {}'.format(
                        label, label_histogram, distance))
                if min_distance is None or distance < min_distance:
                    second_min_distance = min_distance
                    min_distance = distance
                    best_label = label
        if self._printed > 0:
            self._printed -= 1
        ambiguity = 0
        if second_min_distance is not None:
            ambiguity = min_distance / second_min_distance
        confidence = 1 - ambiguity
        return best_label, confidence

    def get_details(self, test_histogram, predicted, actual):
        """Returns details about a classification error.

        Specifically, returns the closest histogram for the predicted item,
        the closest histogram for the actual item, the distance to the
        predicted histogram, and the distance to the actual item as a tuple.

        test_histogram: The histogram whose classification was incorrect.
        predicted: The predicted item.
        actual: The actual item.
        """
        min_predicted = None
        best_predicted = None
        for predicted_histogram in self._data[predicted]:
            distance = self._distance(predicted_histogram, test_histogram)
            if min_predicted is None or distance < min_predicted:
                min_predicted = distance
                best_predicted = predicted_histogram

        min_actual = None
        best_actual = None
        for actual_histogram in self._data[actual]:
            distance = self._distance(actual_histogram, test_histogram)
            if min_actual is None or distance < min_actual:
                min_actual = distance
                best_actual = actual_histogram

        return (best_predicted, best_actual, min_predicted, min_actual)


def read_data(f):
    """Loads training/test set data from a JSON file.

    Returns a dictionary mapping item names to a list of numpy arrays, where
    each numpy array is a color histogram associated with the data.
    """
    json_data = json.load(f)
    item_histograms = {}
    for item, data in json_data.items():
        histograms = data['color_histograms']
        if len(histograms) > 0:
            np_arrays = [np.array(h) for h in histograms]
            item_histograms[item] = np_arrays
    return item_histograms


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


def run_experiment(training_set, test_set, items_in_bin, distance_name):
    """Runs the experiment.

    For each item in the test set, we try all possible combinations of other
    items to put in the bin (items_in_bin is either 2 or 3).

    Running the experiment includes classifying all elements of the test set
    and printing out evaluation results.

    training_set: The training set, from read_data.
    test_set: The set set, from read_data.
    items_in_bin: The number of items in the bin (either 2 or 3).
    distance_name: Either 'l2', 'l1', or 'cosine'
    """
    classifier = None
    print('{} classifier'.format(distance_name))
    if distance_name == 'l2':
        classifier = ColorHistogramClassifier(training_set, l2_distance)
    elif distance_name == 'l1':
        classifier = ColorHistogramClassifier(training_set, l1_distance)
    else:
        classifier = ColorHistogramClassifier(training_set, cosine_distance)

    results = []
    items = [x for x in test_set.keys()]
    for item in items:
        item_histogram = test_set[item][0]

        other_item_generator = None
        if items_in_bin == 2:
            other_item_generator = choose_1_item
        else:
            other_item_generator = choose_2_items

        for other_items in other_item_generator(items, item):
            item_list = [item] + other_items
            predicted, confidence = classifier.classify(item_histogram, item_list)
            results.append((item_histogram, item, predicted, confidence, other_items))

    print_accuracy(results, items_in_bin, distance_name)
    write_results_file(results, items_in_bin, distance_name, classifier)
    write_precision_recall(results, items_in_bin, distance_name, classifier)


def print_accuracy(results, items_in_bin, distance_name):
    """Computes and prints the accuracy results.

    results: The results from run_experiment. A list of (item histogram, item
      name, predicted item name, confidence, list of other item names) tuples.
    items_in_bin: The number of items in the bin for these results.
    distance_name: The distance metric for these results.
    """
    num_correct = 0
    num_correct_by_item = collections.Counter()
    total_by_item = collections.Counter()
    for histogram, item, predicted, confidence, other_items in results:
        if item == predicted:
            num_correct_by_item[item] += 1
            num_correct += 1
        total_by_item[item] += 1
    print('Accuracy ({} items per bin, {} distance): {}'.format(
        items_in_bin, distance_name, num_correct / len(results)))
    for item, num_correct in num_correct_by_item.most_common():
        print('{}\t{}'.format(item, num_correct / total_by_item[item]))


def write_results_file(results, items_in_bin, distance_name, classifier):
    """Writes the results out to a .tsv file.

    results: The results from run_experiment. A list of (item histogram, item
      name, predicted item name, confidence, list of other item names) tuples.
    items_in_bin: The number of items in the bin for these results.
    distance_name: The distance metric for these results.
    classifier: The classifier used for these results.
    """
    output_file = open('{}_{}.tsv'.format(items_in_bin, distance_name), 'w')
    for histogram, item, predicted, confidence, other_items in results:
        correct = 1 if item == predicted else 0
        other_string = '\t'.join(other_items)

        # Show extra columns if the classification was wrong.
        detail = ''
        if predicted != item:
            predicted_histogram, actual_histogram, predicted_distance, actual_distance = classifier.get_details(
                histogram, predicted, item)
            detail = '\t' + '\t'.join(
                [str(histogram), str(predicted_histogram), str(actual_histogram),
                 str(predicted_distance), str(actual_distance)])
        print('{}\t{}\t{}\t{}\t{}{}'.format(item, predicted, correct, confidence, other_string,
                                        detail),
              file=output_file)

def write_precision_recall(results, items_in_bin, distance_name, classifier):
    """Writes the precision-recall curve to a file.

    results: The results from run_experiment. A list of (item histogram, item
      name, predicted item name, confidence, list of other item names) tuples.
    items_in_bin: The number of items in the bin for these results.
    distance_name: The distance metric for these results.
    classifier: The classifier used for these results.
    """
    output_file = open('prcurve_{}_{}.tsv'.format(items_in_bin, distance_name), 'w')
    corrects = [1 if actual == predicted else 0 for _, actual, predicted, _, _ in results]
    confidences = [c for _, _, _, c, _ in results]
    precisions, recalls, thresholds = precision_recall_curve(corrects, confidences)
    for precision, recall, threshold in zip(precisions, recalls, thresholds):
        print('{}\t{}\t{}'.format(precision, recall, threshold), file=output_file)
        

if __name__ == '__main__':
    training_set_file = open(sys.argv[1])
    test_set_file = open(sys.argv[2])
    training_set = read_data(training_set_file)
    test_set = read_data(test_set_file)
    #run_experiment(training_set, test_set, 2, 'l2')
    run_experiment(training_set, test_set, 2, 'l1')
    #run_experiment(training_set, test_set, 2, 'cosine')
    #run_experiment(training_set, test_set, 3, 'l2')
    run_experiment(training_set, test_set, 3, 'l1')
    #run_experiment(training_set, test_set, 3, 'cosine')
