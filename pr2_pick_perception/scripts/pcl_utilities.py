#!/usr/bin/env python

from __future__ import division
from geometry_msgs.msg import Point, PointStamped
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf

from pr2_pick_perception.srv import BoxPoints, BoxPointsResponse 
from pr2_pick_perception.srv import FindCentroid, FindCentroidResponse

class PCLUtilities(object):
    '''Computes and logs the time taken to process a point cloud. '''

    def __init__(self):
        self._find_centroid_service = rospy.Service(
            'perception/find_centroid',
            FindCentroid,
            self.find_centroid
        )

        self._points_in_box_service = rospy.Service(
            'perception/points_in_box',
            BoxPoints,
            self.find_points_in_box
        )

    def find_centroid(self, request):
        '''Computes the average point in a point cloud. '''
        points = pc2.read_points(
            request.cluster.pointcloud,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
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

    def find_points_in_box(self, request):
        ''' Returns number of points within bounding box specified by 
            request. '''        
        points = pc2.read_points(
            request.cluster.pointcloud,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )

        num_points = 0
        tf_listener = tf.TransformListener()
        for x, y, z in points:

            # Transform point into frame of bounding box
            point = PointStamped(
                point=Point(x=x, y=y, z=z),
                header=Header(
                    frame_id=request.cluster.header.frame_id,
                    stamp=rospy.Time.now(),
                )
            )

            transformed_point = tf_listener.transformPose('r_wrist_roll_link',
                                                                point)
            if (transformed_point.point.x => request.min_x and
                transformed_point.point.x <= request.max_x and
                transformed_point.point.y => request.min_y and
                transformed_point.point.y <= request.max_y and
                transformed_point.point.z => request.min_z and
                transformed_point.point.z <= request.max_z):
                num_points += 1

        return BoxPointsResponse(num_points=num_points)


def main():
    rospy.init_node('pcl_utilities');
    utils = PCLUtilities()
    rospy.spin()


if __name__ == '__main__':
    main()
