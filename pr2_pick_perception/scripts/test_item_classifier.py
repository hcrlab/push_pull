from item_classifier import TargetItemClassifier
import mock
import unittest

class TestTargetItemClassifier(unittest.TestCase):
    def test_simple(self):
        """Simple case - one descriptor is labeled as the target, another is
        not. We should return the target item even if the confidence is lower.
        """
        def classify(descriptor, possible_labels):
            if descriptor == 0:
                return 'a', 0.5
            else:
                return 'b', 1
        classifier = mock.Mock()
        classifier.classify = classify
        target_item_classifier = TargetItemClassifier(classifier)
        i, confidence = target_item_classifier.classify([0, 1], 'a', ['a', 'b'])
        self.assertEqual(i, 0)
        self.assertEqual(confidence, 0.5)

    def test_single_item_bin(self):
        """A single item bin, should return the 0 index with confidence 1."""
        def classify(descriptor, possible_labels):
            return 'b', 0.5
        classifier = mock.Mock()
        classifier.classify = classify
        target_item_classifier = TargetItemClassifier(classifier)
        i, confidence = target_item_classifier.classify([1], 'a', ['a'])
        self.assertEqual(i, 0)
        self.assertEqual(confidence, 1)


    def test_all_not_target_item(self):
        """When the classifier doesn't label any of the descriptors as the
        target item, then we accept the most confident prediction as true, and
        try classifying again. In this case, 0 is the target item, but both
        descriptors are initially classified as 'b'.
        """
        def classify(descriptor, possible_labels):
            if descriptor == 0:
                if 'b' in possible_labels:
                    return 'b', 0.8
                else:
                    return 'a', 0.5
            else:
                if 'b' in possible_labels:
                    return 'b', 1
                else:
                    return 'a', 0.2
        classifier = mock.Mock()
        classifier.classify = classify
        target_item_classifier = TargetItemClassifier(classifier)
        i, confidence = target_item_classifier.classify([0, 1], 'a', ['a', 'b'])
        self.assertEqual(i, 0)
        self.assertEqual(confidence, 0.5)


    def test_all_not_target_item(self):
        """When the classifier doesn't label any of the descriptors as the
        target item, then we accept the most confident prediction as true, and
        try classifying again. In this case, 0 is the target item, but both
        descriptors are initially classified as 'b'.
        """
        def classify(descriptor, possible_labels):
            if descriptor == 0:
                if 'b' in possible_labels:
                    return 'b', 0.8
                else:
                    return 'a', 0.5
            else:
                if 'b' in possible_labels:
                    return 'b', 1
                else:
                    return 'a', 0.2
        classifier = mock.Mock()
        classifier.classify = classify
        target_item_classifier = TargetItemClassifier(classifier)
        i, confidence = target_item_classifier.classify([0, 1], 'a', ['a', 'b'])
        self.assertEqual(i, 0)
        self.assertEqual(confidence, 0.5)


    def test_multiple_target_items(self):
        """When the classifier labels more than one descriptor as the target
        item, then we should pick the most confident one.
        """
        def classify(descriptor, possible_labels):
            if descriptor == 0:
                if 'a' in possible_labels:
                    return 'a', 0.8
                else:
                    return 'b', 0.5
            else:
                if 'a' in possible_labels:
                    return 'a', 1
                else:
                    return 'b', 0.2
        classifier = mock.Mock()
        classifier.classify = classify
        target_item_classifier = TargetItemClassifier(classifier)
        i, confidence = target_item_classifier.classify([0, 1], 'a', ['a', 'b'])
        self.assertEqual(i, 1)
        self.assertEqual(confidence, 1)


    def test_less_descriptors_than_labels(self):
        """If for some reason the number of descriptors is less than the number
        of labels, then try to classify them anyway.
        """
        def classify(descriptor, possible_labels):
            if descriptor == 0:
                if 'b' in possible_labels:
                    return 'b', 0.8
                else:
                    return 'a', 0.5
            else:
                if 'b' in possible_labels:
                    return 'b', 1
                else:
                    return 'c', 0.2
        classifier = mock.Mock()
        classifier.classify = classify
        target_item_classifier = TargetItemClassifier(classifier)
        i, confidence = target_item_classifier.classify([0, 1], 'a', ['a', 'b', 'c'])
        self.assertEqual(i, 0)
        self.assertEqual(confidence, 0.5)

        

if __name__ == '__main__':
    unittest.main()
