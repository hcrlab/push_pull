from __future__ import division
from __future__ import print_function

from pr2_pick_perception.msg import ItemDescriptor
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neighbors import NearestNeighbors
import numpy as np
import rospy


class ItemClassifier(object):
    def __init__(self, training_data, normalize=False):
        """Constructor.

        training_data: A list of (descriptor, label), where descriptor is a
            pr2_pick_perception/ItemDescriptor, and label is a string item
            name.
        """
        self._normalize = normalize
        self._data_by_class = self._load_data(training_data)

    def _load_data(self, data):
        data_by_class = {}
        for descriptor, label in data:
            histogram = np.array(descriptor.histogram.histogram)
            if self._normalize:
                histogram = histogram / sum(histogram)
            if label in data_by_class:
                data_by_class[label].append(histogram)
            else:
                data_by_class[label] = [histogram]
        return data_by_class

    def _find_nearest_with_label(self, histogram, label):
        nn = NearestNeighbors(n_neighbors=1)
        data = self._data_by_class[label]
        nn.fit(data)
        dist, ind = nn.kneighbors([histogram])
        return data[ind[0][0]], dist[0][0]

    def _sorted_points(self, descriptor, labels):
        if len(labels) == 0:
            raise rospy.ServiceException(
                'Can\'t classify item with no labels.')

        # Find nearest point of each class.
        histogram = np.array(descriptor.histogram.histogram)
        if self._normalize:
            histogram = histogram / sum(histogram)
        points = []
        for label in labels:
            point, distance = self._find_nearest_with_label(histogram, label)
            points.append((point, distance, label))

        sorted_points = sorted(points, key=lambda x: x[1])
        return sorted_points

    def compute_confidences(self, descriptor, labels):
        """Get the confidence for each label in labels.
        Returns a sorted list of (label, confidence).
        """
        sorted_points = self._sorted_points(descriptor, labels)
        distances = [d for p, d, l in sorted_points]
        total = sum(distances)
        confidences = np.array([1 - d / total for p, d, l in sorted_points])
        confidences /= sum(confidences)
        labels = [l for p, d, l in sorted_points]
        return labels, list(confidences)

    def classify(self, descriptor, labels):
        """Returns the most likely label assignment for the given descriptor,
        and a confidence for that label.

        descriptor: The descriptor to classify, a pr2_pick_perception/ItemDescriptor.
        labels: A list of item names to possibly classify the descriptor as.
        """
        labels, confidences = self.compute_confidences(descriptor, labels)
        return labels[0], confidences[0]


if __name__ == '__main__':
    d1 = ItemDescriptor()
    d1.histogram.histogram = [0, 1, 2, 3, 4, 5, 6, 7]
    d2 = ItemDescriptor()
    d2.histogram.histogram = [1, 1, 2, 3, 4, 5, 6, 7]
    d3 = ItemDescriptor()
    d3.histogram.histogram = [7, 6, 5, 4, 3, 2, 1, 0]
    d4 = ItemDescriptor()
    d4.histogram.histogram = [6, 6, 5, 4, 3, 2, 1, 0]
    data = [(d1, 'cat'), (d2, 'cat'), (d3, 'wolf'), (d4, 'dog')]
    item_classifier = ItemClassifier(data)
    label, prob = item_classifier.classify(d3, ['cat', 'dog'])
    # Expect the "wolf" to be classified as a "dog"
    print(label, prob)
