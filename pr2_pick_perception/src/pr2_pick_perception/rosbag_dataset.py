from __future__ import division
from __future__ import print_function

import os
import rosbag


class RosbagDataset(object):
    def __init__(self, data_dir):
        """Loads a rosbag dataset.

        All the rosbags are in data_dir. Each rosbag has a topic called cropped_cloud
        of type pr2_pick_perception/MultiItemCloud.
        """
        self._data_dir = data_dir
        self._data = []
        self._load()

    def _load(self):
        filenames = os.listdir(self._data_dir)
        for filename in filenames:
            bag = rosbag.Bag(os.path.join([self._data_dir, filename]))
            for topic, msg, time in bag.read_messages(topics=['cropped_cloud']):
                self._data.append(msg)
            bag.close()

    def data(self):
        """Returns a list of pr2_pick_perception/MultiItemCloud
        """
        return self._data
