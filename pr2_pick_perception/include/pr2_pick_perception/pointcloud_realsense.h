#ifndef _PR2_PICK_PERCEPTION_POINT_CLOUD_REALSENSE_H_
#define _PR2_PICK_PERCEPTION_POINT_CLOUD_REALSENSE_H_

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>

#include <sensor_msgs/point_cloud_conversion.h>


namespace pr2_pick_perception {

class PointCloudRealsense
{
public:
    PointCloudRealsense(){};
    ~PointCloudRealsense(){};
    
    bool init();
    
    
private:
    
    image_transport::SubscriberFilter im_color_sub_;
    image_transport::SubscriberFilter im_depth_sub_;
    
    image_geometry::PinholeCameraModel colorproj_, depthproj_; 
    
    
    ros::Publisher pc_pub_;
    cv::Size color_size_, depth_size_;
   
    /// camera reference system    
    std::string color_frame_id_, depth_frame_id_;
    
    cv::Mat Kdepth_,R_,T_,Kcolor_;
    cv::Mat Kdepth_und_,Kcolor_und_;
    
    cv::Mat map_depth_x_,map_depth_y_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ASyncPolicy;
    typedef message_filters::Synchronizer<ASyncPolicy> Async;
    
    boost::shared_ptr<Async>sync_;
    
    void pointcloudCallBack(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPC(cv::Mat depth, cv::Mat color);
};
};
#endif