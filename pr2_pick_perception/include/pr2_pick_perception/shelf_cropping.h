#ifndef CROP_SHELF
#define CROP_SHELF

#include <string>

// ROS
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>

// TF
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
// PCL ROS

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>

// picking
#include <pr2_pick_perception/CropShelf.h>
#include <pr2_pick_perception/Cell.h>
#include <pr2_pick_perception/ObjectList.h>

// markers
#include <visualization_msgs/Marker.h>

class CropShelf {
 public:
  CropShelf();
  ~CropShelf() {}

  ///\brief initialize shelf cropper
  ///\return false if the pose of the shelf is not available
  bool initialize();
  bool cropCallBack(pr2_pick_perception::CropShelfRequest &request,
                    pr2_pick_perception::CropShelfResponse &response);

 private:
  // enum class CelID  {A, B, C, D, E,F,G,H,I,J,K,L };
  /// cell ID to retrieve
  std::string cell_id_;
  // point cloud read successfully
  bool pc_ready_;
  // shelf pose available
  bool shelf_pose_;
  // kinect mutex
  boost::mutex kinect_mtx_;

  /// refence transform listener
  tf::TransformListener tf_;

  /// cloud to robot transform
  tf::StampedTransform cloud_to_robot_;
  /// robot to world transform
  tf::StampedTransform robot_to_world_;
  /// shelf to left top corner origin
  tf::StampedTransform shelf_to_origin_;
  /// cloud to bin
  tf::StampedTransform cloud_to_bin_;

  /// shelf transform
  tf::Transform shelf_transform_;

  /// robot refence system
  std::string robot_frame_id_;
  /// world reference system
  std::string world_frame_id_;
  /// lidar reference system
  std::string cloud_frame_id_;
  /// shelf reference system
  std::string shelf_frame_id_;
  /// model reference system
  std::string bin_frame_id_;

  bool debug_;

  double cell_width1_, cell_width2_, cell_height1_, cell_height2_, depth_cell_;
  double bottom_crop_offset_, top_crop_offset_, left_crop_offset_,
      right_crop_offset_, depth_close_crop_offset_, depth_far_crop_offset_;
  int max_cluster_points_, min_cluster_points_;

  /// subscriber to trigger
  ros::Subscriber trigger_sub_;
  /// subscriber to point cloud
  ros::Subscriber pc_sub_;

  ros::Subscriber shelf_pose_sub_;

  ros::ServiceServer server_;

  ros::NodeHandle nh_;
  // original point cloud
  pcl::PointCloud<pcl::PointXYZRGB> kinect_pc_;

  sensor_msgs::PointCloud2Ptr kinect_pc_ros_;
  ros::Time pc_timestamp_;

  void poseListener(pr2_pick_perception::ObjectList shelfdetection);

  void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &pcloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropPC(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &shelf_pc, float width,
      float height, float depth, int cellID);

  ros::Publisher vis_pub_;
  void visualizeShelf(float width, float height, float depth);
};

#endif
