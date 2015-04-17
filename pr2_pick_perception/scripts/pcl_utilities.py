#!/usr/bin/env python

from __future__ import division
from geometry_msgs.msg import PointStamped
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class PCLUtilities(object):
    '''Computes and logs the time taken to process a point cloud. '''

    def __init__(self, prev_time):
        self._find_centroid_service = rospy.Service(
            'perception/find_centroid',
            FindCentroid,
            self.find_centroid
        )

    def find_centroid(self, cluster):
        '''Computes the average point in a point cloud. '''
        points = pc2.read_points(cluster.pointcloud, data, field_names=['x', 'y', 'z'],
            skip_nans=True)

        num_points = 0
        avg_x = 0
        avg_y = 0
        avg_z = 0
        for x, y, z in points:
            num_points += 1
            avg_x += x
            avg_y += y
            avg_z += z
        if num_points > 0:
            avg_x /= num_points
            avg_y /= num_points
            avg_z /= num_points

        rospy.loginfo('Centroid: ({}, {}, {})'.format(avg_x, avg_y, avg_z))
        response = PointStamped(
            point=Point(x=avg_x, y=avg_y, z=agv_z),
            header=Header(
                frame_id=cluster.header.frame_id,
                stamp=rospy.Time.now(),
            )
        )
        return response


def main():
    rospy.init_node('pcl_utilities');
    processor = Processor(rospy.Time.now())
    rospy.spin()


if __name__ == '__main__':
    main()
