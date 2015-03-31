#ifndef RGB_PC_H_
#define RGB_PC_H_


#include <ros/ros.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <math.h>

typedef pcl::PointXYZ WPoint;
typedef pcl::PointCloud<WPoint> WPointCloud;

pcl::PointCloud<pcl::PointXYZRGB> rgbpc;

ros::Publisher rgbpc_pub;

bool image_ready = false;

image_geometry::PinholeCameraModel leftcamproj;

tf::StampedTransform Tlidar2cam;

//static cv::Mat imageLeft;

sensor_msgs::ImageConstPtr l_image_msg;
int width;
int height;

std::string lidar_frame_id;
std::string camera_frame_id;

cv_bridge::CvImageConstPtr l_cv_ptr;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg);

void processImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  
#endif