from __future__ import division
from __future__ import print_function

from pr2_pick_perception import ItemClassifier
from pr2_pick_perception.msg import ItemDescriptor
from pr2_pick_perception.srv import ClassifyTargetItemResponse
import numpy as np
import rospy


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

        target_confidences = []
        for descriptor in descriptors:
            labels, confidences = self._item_classifier.compute_confidences(
                descriptor, all_items)
            target_index = labels.index(target_item)
            target_confidence = confidences[target_index]
            target_confidences.append(target_confidence)

        max_confidence = None
        max_index = None
        for i, tc in enumerate(target_confidences):
            if max_confidence is None or tc > max_confidence:
                max_confidence = tc
                max_index = i

        return max_index, max_confidence

    def classify_request(self, request):
        target_item_index, confidence = self.classify(request.descriptors,
                                                      request.target_item,
                                                      request.all_items)
        response = ClassifyTargetItemResponse()
        response.target_item_index = target_item_index
        response.confidence = confidence
        return response


if __name__ == '__main__':
    t1 = ItemDescriptor()
    t1.histogram.histogram = [0]
    t2 = ItemDescriptor()
    t2.histogram.histogram = [1]
    t3 = ItemDescriptor()
    t3.histogram.histogram = [10]
    t4 = ItemDescriptor()
    t4.histogram.histogram = [11]
    data = [(t1, 'cat'), (t2, 'cat'), (t3, 'dog'), (t4, 'dog')]
    item_classifier = ItemClassifier(data)
    target_item_classifier = TargetItemClassifier(item_classifier)
    d3 = ItemDescriptor()
    d3.histogram.histogram = [9]
    d4 = ItemDescriptor()
    d4.histogram.histogram = [8]
    ind, prob = target_item_classifier.classify([d3, d4], 'cat', ['cat',
                                                                  'dog'])
    # Expect that the index is 1.
    print(ind, prob)
