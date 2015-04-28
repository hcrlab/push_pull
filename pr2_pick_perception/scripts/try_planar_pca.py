#!/usr/bin/env python

import rospy
import rosbag
import sys
from pr2_pick_perception.msg import Cluster
from pr2_pick_perception.srv import PlanarPrincipalComponents


def get_first_pc(rosbag_path):
    bag = rosbag.Bag(rosbag_path)
    for topic, msg, t in bag.read_messages('/head_mount_kinect/depth_registered/points'):
        return msg


def main():
    rosbag_path = sys.argv[1]
    pc = get_first_pc(rosbag_path)
    cluster = Cluster()
    cluster.header.frame_id = pc.header.frame_id
    cluster.pointcloud = pc
    get_planar_pca = rospy.ServiceProxy('planar_principal_components',
                                        PlanarPrincipalComponents)
    get_planar_pca.wait_for_service()
    response = get_planar_pca(cluster)
    rospy.loginfo(response)


if __name__ == '__main__':
    rospy.init_node('try_planar_pca')
    main()
