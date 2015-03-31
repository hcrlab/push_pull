#ifndef SELECT_OBJECT_H_
#define SELECT_OBJECT_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/package.h>


#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/String.h>

#include <laser_assembler/AssembleScans.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>


#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>


#include <trooper_adaptive_perception_msgs/SelectObject.h>

typedef pcl::PointXYZ PointT;


class Obj_selector{

public:
  void initialize();
  void start();
  

   Obj_selector();
  ~Obj_selector(){}

  
private:
  tf::TransformListener tf_;
  ros::ServiceClient client_;
  /// stereo to world transform 
  tf::StampedTransform stereo2world_;
  /// cloud to robot transform
  tf::StampedTransform cloud_to_robot_;
  /// cloud to camera
  tf::StampedTransform cloud_to_camera_;
    
    
  std::string robot_frame_id_;
  /// world reference system
  std::string world_frame_id_;
  /// stereo reference system
  std::string stereo_ref_system_;
  /// lidar reference system
  std::string cloud_frame_id_;
  /// camera reference system
  std::string camera_frame_id_;
    
  boost::mutex stereo_mtx_; 
  
  cv_bridge::CvImageConstPtr l_cv_ptr_;
  
  bool image_ready_;
  bool flag_callback_;
  bool flag_service_;
  

  image_geometry::PinholeCameraModel leftcamproj_;

  tf::StampedTransform Tlidar2cam_;
  ros::Time pc_stamp_;
  sensor_msgs::ImageConstPtr l_image_msg_;
  pcl::PointCloud<pcl::PointXYZ> stereo_pcl_;
  
  cv::Mat frame_;
  /// Image to send to the user
  std::vector<cv::Mat> HSV_;

  /// synchronization 
  /// image subscriber
  image_transport::SubscriberFilter image_sub_;
  /// camera info subscriber   
  message_filters::Subscriber<sensor_msgs::CameraInfo> im_info_sub_;
  /// stereo point cloud subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> stereo_pcl_sub_;   
  
  ros::ServiceServer select_object_pub_;
    
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  boost::shared_ptr<ExactSync> exact_sync_;    
  
  ros::NodeHandle nh_;
  
  
  
  /// for sync checking
  static void increment(int* value) {  ++(*value);  }
  /// checks for synchronized data streams, gets called every 15 seconds
  void checkInputsSynchronized();


  /// Countger synchronization variables
  int image_received_, im_info_received_, stereo_pcl_received_, all_received_;    
  // for sync checking
  ros::Timer check_synced_timer_;
  
  
    /// synchronization wrapper for data callback
  void dataCbSync(const sensor_msgs::ImageConstPtr &img_msg, 
			const sensor_msgs::CameraInfoConstPtr &cam_info,                           
			const sensor_msgs::PointCloud2ConstPtr &pcl_msg);



  /// callback function for receiving image and stereo point cloud
  void dataCallback( const sensor_msgs::ImageConstPtr &img_msg, 
		const sensor_msgs::CameraInfoConstPtr &cam_info,                       
		const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
  
  
  /// service publisher
  bool selectObjectCb( trooper_adaptive_perception_msgs::SelectObject::Request &request, 
		  trooper_adaptive_perception_msgs::SelectObject::Response &response);
  

};
#endif