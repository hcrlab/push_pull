#ifndef CROP_IMAGE_H_
#define CROP_IMAGE_H_


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

#include <adaptive_perception_msgs/ObjectDetectionRequest.h>

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
ros::Publisher roi_pub;
cv::Mat frame;
const char* src_window_ = "Select ROI";
std::string objt_name;
int drag = 0, select_flag = 0;

//points of mouse callback
cv::Point2f point1, point2;
bool callback_im = false;
adaptive_perception_msgs::ObjectDetectionRequest message;
std::string lidar_frame_id;
std::string camera_frame_id;

cv_bridge::CvImageConstPtr l_cv_ptr;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg);

void processImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

void callbackmouse(int event, int x, int y, int flags, void* param);

// bool cropCallback( trooper_adaptive_perception_msgs::SelectObjectImg::Request &request, 
// 		  trooper_adaptive_perception_msgs::SelectObjectImg::Response &response);
#endif
