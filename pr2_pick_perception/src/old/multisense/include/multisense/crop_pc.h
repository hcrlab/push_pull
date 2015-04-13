#ifndef RGB_PC_H_
#define RGB_PC_H_


#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <math.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <ros/node_handle.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include <adaptive_perception_msgs/GetPointCloudROI.h>

typedef pcl::PointXYZ WPoint;
typedef pcl::PointCloud<WPoint> WPointCloud;

WPointCloud pc_crop,pc_in;

ros::ServiceServer croppc_pub;

bool image_ready = false;
bool pc_ready    = false;

image_geometry::PinholeCameraModel leftcamproj;

tf::StampedTransform Tlidar2cam;
ros::Time pc_stamp;
sensor_msgs::ImageConstPtr l_image_msg;
int width;
int height;

cv::Mat frame;
const char* src_window = "Select ROI";

int drag = 0, select_flag = 0;

//points of mouse callback
cv::Point point1, point2;
bool callback_im = false;

std::string lidar_frame_id;
std::string camera_frame_id;

cv_bridge::CvImageConstPtr l_cv_ptr;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg);

void processImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

void callbackmouse(int event, int x, int y, int flags, void* param);

bool cropCallback( adaptive_perception_msgs::GetPointCloudROI::Request &request,
		  adaptive_perception_msgs::GetPointCloudROI::Response &response);
#endif
