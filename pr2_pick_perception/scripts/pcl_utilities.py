#!/usr/bin/env python

from __future__ import division
from geometry_msgs.msg import PointStamped
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

from pr2_pick_perception.srv import FindCentroid, FindCentroidResponse

class PCLUtilities(object):
    '''Computes and logs the time taken to process a point cloud. '''

    def __init__(self):
        self._find_centroid_service = rospy.Service(
            'perception/find_centroid',
            FindCentroid,
            self.find_centroid
        )

    def find_centroid(self, request):
        '''Computes the average point in a point cloud. '''
        points = pc2.read_points(
            request.cluster.pointcloud,
            data,
            field_names=['x', 'y', 'z'],
            skip_nans=True
        )

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
        centroid = PointStamped(
            point=Point(x=avg_x, y=avg_y, z=avg_z),
            header=Header(
                frame_id=request.cluster.header.frame_id,
                stamp=rospy.Time.now(),
            )
        )
        return FindCentroidResponse(centroid=centroid)


def main():
    rospy.init_node('pcl_utilities');
    utils = PCLUtilities()
    rospy.spin()


if __name__ == '__main__':
    main()
