#!/bin/bash
rosbag record --duration 20 /tilt_scan_cloud /head_mount_kinect/depth_registered/points /tf /head_mount_kinect/depth_registered/camera_info /head_mount_kinect/rgb/camera_info /head_mount_kinect/rgb/image_rect_color
