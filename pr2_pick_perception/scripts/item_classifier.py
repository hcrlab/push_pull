#!/usr/bin/env python
"""Classifies objects based on descriptors.
"""
from __future__ import division
from __future__ import print_function
from pr2_pick_perception.srv import ClassifyTargetItem, ClassifyTargetItemResponse
import argparse
import json
import numpy as np
import rospy


def l2_distance(h1, h2):
    return np.linalg.norm(h1 - h2)


def l1_distance(h1, h2):
    return np.linalg.norm(h1 - h2, ord=1)


def cosine_distance(h1, h2):
    return h1.dot(h2) / (np.linalg.norm(h1) * np.linalg.norm(h2))


class ItemClassifier(object):
    def __init__(self, training_data):
        """Constructor.

        training_data: A map from item names to a dictionary of data.
        """
        self._data = training_data
        self._distance = l1_distance

    def classify(self, descriptor, labels):
        """Returns the most likely label assignment for the given descriptor,
        and a confidence for that label.

        descriptor: The descriptor to classify, a pr2_pick_perception/ItemDescriptor.
        labels: A list of item names to possibly classify the descriptor as.
        """
        histogram = np.array(descriptor.histogram.histogram)
        if len(labels) == 0:
            raise rospy.ServiceException('Can\'t classify item with no labels.')
        second_min_distance = None
        min_distance = None
        best_label = None
        for label in labels:
            item_data = self._data[label]
            label_histograms = [np.array(x)
                                for x in item_data['color_histograms']]
            for label_histogram in label_histograms:
                distance = self._distance(histogram, label_histogram)
                if min_distance is None or distance < min_distance:
                    second_min_distance = min_distance
                    min_distance = distance
                    best_label = label
        ambiguity = 0
        if second_min_distance is not None:
            ambiguity = min_distance / second_min_distance
        confidence = 1 - ambiguity
        return best_label, confidence


class TargetItemClassifier(object):
    def __init__(self, item_classifier):
        self._item_classifier = item_classifier

    def classify(self, descriptors, target_item, all_items):
        """Identifies the target item from a list of descriptors.

        descriptors: The list of descriptors to choose from.
        target_item: The name of the target item.
        all_items: The names of all items in the bin.

        Returns: the index of the target item in the descriptors list, and a
        confidence score.
        """
        # Handle edge cases.
        if len(descriptors) != len(all_items):
            rospy.logwarn(
                '[ItemClassifier]: # of descriptors ({}) did not match # of labels ({}).'.format(
                    len(descriptors), len(all_items)))
        if len(descriptors) == 0:
            rospy.logwarn(
                '[ItemClassifier]: no descriptors passed to classify!')
            return 0, 0
        if len(all_items) == 0:
            rospy.logwarn('[ItemClassifier]: empty bin passed to classify!')
            return 0, 0
        if len(all_items) == 1:
            return 0, 1
        if len(descriptors) == 1:
            return 0, 1

        possible_descriptors = [(i, x) for (i, x) in enumerate(descriptors)]
        possible_labels = [x for x in all_items]
        while True:
            target_labels = []
            most_confident_index = None
            most_confident_label = None
            highest_confidence = None
            if len(possible_descriptors) == 0:
                raise rospy.ServiceException('No possible descriptors.')
            for i, descriptor in possible_descriptors:
                label, confidence = self._item_classifier.classify(
                    descriptor, possible_labels)
                rospy.loginfo(
                    'Classified cluster {} as {} with confidence {}'.format(
                        i, label, confidence))
                if highest_confidence is None or confidence > highest_confidence:
                    highest_confidence = confidence
                    most_confident_label = label
                    most_confident_index = i
                if label == target_item:
                    target_labels.append((i, confidence))
            if len(target_labels) > 0:
                sorted_target_labels = sorted(target_labels,
                                              key=lambda x: x[1],
                                              reverse=True)
                return sorted_target_labels[0]
            # Otherwise, accept the most confident label as true, and narrow
            # down the set of possible labels.
            if most_confident_label in possible_labels:
                possible_labels.remove(most_confident_label)
            else:
                rospy.logerr('Most confident label {} not possible! {}'.format(
                    most_confident_label, possible_labels))
                raise rospy.ServiceException('Most confident label {} not possible! {}, {}'.format(
                    most_confident_label, possible_labels, possible_descriptors))
            possible_descriptors = (
                possible_descriptors[:i] + possible_descriptors[i + 1:]
            )

    def classify_request(self, request):
        target_item_index, confidence = self.classify(request.descriptors,
                                                      request.target_item,
                                                      request.all_items)
        response = ClassifyTargetItemResponse()
        response.target_item_index = target_item_index
        response.confidence = confidence
        return response


if __name__ == '__main__':
    rospy.init_node('item_classifier')

    parser = argparse.ArgumentParser()
    parser.add_argument('filename',
                        metavar='FILE',
                        type=str,
                        help='JSON file specifying item data')
    args = parser.parse_args(args=rospy.myargv()[1:])

    item_data = json.load(open(args.filename, 'r'))
    item_classifier = ItemClassifier(item_data)
    target_item_classifier = TargetItemClassifier(item_classifier)

    rospy.Service('item_classifier/classify_target_item', ClassifyTargetItem,
                  target_item_classifier.classify_request)

    rospy.spin()
