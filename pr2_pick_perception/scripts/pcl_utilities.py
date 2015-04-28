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
            'perception/get_points_in_box',
            BoxPoints,
            self.find_points_in_box
        )
        self._tf_listener = tf.TransformListener()

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

        num_points = []
        for x, y, z in points:

            # Transform point into frame of bounding box
            point = PointStamped(
                point=Point(x=x, y=y, z=z),
                header=Header(
                    frame_id=request.cluster.header.frame_id,
                    stamp=rospy.Time(0),
                )
            )

            #self._tf_listener.waitForTransform(request.cluster.header.frame_id, 
            #                                request.frame_id, rospy.Time(0),
            #                                rospy.Duration(10.0))

            transformed_point = self._tf_listener.transformPoint(request.frame_id,
                                                                point)

            for (idx, box) in enumerate(request.boxes):
                if (transformed_point.point.x >= box.min_x and
                    transformed_point.point.x <= box.max_x and
                    transformed_point.point.y >= box.min_y and
                    transformed_point.point.y <= box.max_y and
                    transformed_point.point.z >= box.min_z and
                    transformed_point.point.z <= box.max_z):
                    num_points[idx] += 1

        return BoxPointsResponse(num_points=num_points)


def main():
    rospy.init_node('pcl_utilities');
    utils = PCLUtilities()
    rospy.spin()


if __name__ == '__main__':
    main()
