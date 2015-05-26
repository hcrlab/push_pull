import os
import rosbag
import rospy

class DataSaver(object):
    def __init__(self, data_dir, filename):
        self._data_dir = data_dir
        self._filename = filename
        self._data_path = os.path.join([data_dir, filename])
        self._is_bag_open = False
        if len(os.listdir(data_dir)) > 12:
            rospy.logwarn('Bag directory {} has more than 10 files in it, not saving data.')
            return
        self._bag = rosbag.Bag(data_path, 'w')
        self._is_bag_open = True

    def save_message(self, topic, message):
        if not self._is_bag_open:
            rospy.logwarn('Bag file {} is already closed, not writing data.'.format(self._filename))
            return
        self._bag.write(topic, message)

    def close():
        self._bag.close()
        self._is_bag_open = False
