#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pr2_pick_perception/shelf_cropping.h>

// markers
#include <visualization_msgs/Marker.h>
#include <ros/topic.h>
#include <ros/ros.h>

using pcl::PointCloud;
using pcl::PointXYZRGB;

namespace pr2_pick_perception {
ShelfCropper::ShelfCropper() {}

bool ShelfCropper::initialize() {
  ros::NodeHandle nh_local("~");
  ros::NodeHandle nh;

  // subscribe to point cloud topic
  pc_topic_ = nh.resolveName("/pc_topic");

  nh_local.param("Debug", debug_, false);

  nh_local.param("Width1", cell_width1_, 0.2671);
  nh_local.param("Width2", cell_width2_, 0.2991);
  nh_local.param("Height1", cell_height1_, 0.26);
  nh_local.param("Height2", cell_height2_, 0.23);
  nh_local.param("Depth", depth_cell_, 0.43);

  // Parameters for tweaking shelf crop
  nh_local.param("bottom_crop_offset", bottom_crop_offset_, 0.02);
  nh_local.param("top_crop_offset", top_crop_offset_, 0.02);
  nh_local.param("left_crop_offset", left_crop_offset_, 0.08);
  nh_local.param("right_crop_offset", right_crop_offset_, 0.02);
  nh_local.param("depth_close_crop_offset", depth_close_crop_offset_, 0.02);
  nh_local.param("depth_far_crop_offset", depth_far_crop_offset_, 0.02);

  vis_pub_ =
      nh.advertise<visualization_msgs::Marker>("/pr2_pick_visualization", 0);

  return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShelfCropper::cropPC(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &shelf_pc, float width,
    float height, float depth, int cellID) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cell_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  ROS_INFO("Cropping PC, num points: %d", shelf_pc->points.size());
  float near_extension = 0.1;
  float bottom_lift = 0.02;
  for (size_t i = 0; i < shelf_pc->points.size(); i++) {
    pcl::PointXYZRGB point = shelf_pc->points[i];
    // The point cloud has been transformed around the frame of the bin.
    // The origin of the bin's frame is in the front bottom center of the bin.
    // A second box that includes points slightly in front of the bin.
    if (point.x < (depth - depth_far_crop_offset_) &&
        point.x >= (0 + depth_close_crop_offset_) &&
        point.y < (width / 2 - left_crop_offset_) &&
        point.y >= (-width / 2 + right_crop_offset_) &&
        point.z < (height - top_crop_offset_) &&
        point.z >= (0 + bottom_crop_offset_)) {
      cell_pc->push_back(point);
    } else if (point.x < (depth - depth_far_crop_offset_) &&
               point.x >= (depth_close_crop_offset_ - near_extension) &&
               point.y < (width / 2 - left_crop_offset_) &&
               point.y >= (-width / 2 + right_crop_offset_) &&
               point.z < (height - top_crop_offset_) &&
               point.z >= (bottom_lift + bottom_crop_offset_)) {
      cell_pc->push_back(point);
    }
  }
  return cell_pc;
}

// Given width, height, and depth of a shelf and the id for the shelf, publishes
// a marker demonstrating the area considered to be part of the shelf. This area
// exactly matches the point cloud cropping of ShelfCropper::cropPC.
void ShelfCropper::visualizeShelf(float width, float height, float depth) {
  // crop_* is the dimensions of the crop box.
  float crop_depth = depth - depth_far_crop_offset_ - depth_close_crop_offset_;
  float crop_width = width - left_crop_offset_ - right_crop_offset_;
  float crop_height = height - top_crop_offset_ - bottom_crop_offset_;

  visualization_msgs::Marker marker;
  marker.header.frame_id = bin_frame_id_;
  marker.header.stamp = ros::Time();
  marker.ns = "crop_shelf_marker";
  marker.id = 2 * bin_frame_id_[bin_frame_id_.length() - 1];
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = crop_depth / 2 + depth_close_crop_offset_;
  marker.pose.position.y = (right_crop_offset_ - left_crop_offset_) / 2;
  marker.pose.position.z = crop_height / 2 + bottom_crop_offset_;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = crop_depth;
  marker.scale.y = crop_width;
  marker.scale.z = crop_height;
  marker.color.a = 0.2;
  marker.color.r = 0.9;
  marker.color.g = 0.9;
  marker.color.b = 0.9;
  marker.lifetime = ros::Duration(0);

  // Second box, which extends towards the robot more, and is lifted up more.
  float near_extension = 0.1;
  float bottom_lift = 0.02;

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = bin_frame_id_;
  marker2.header.stamp = ros::Time();
  marker2.ns = "crop_shelf_marker";
  marker2.id = 2 * bin_frame_id_[bin_frame_id_.length() - 1] + 1;
  marker2.type = visualization_msgs::Marker::CUBE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = (crop_depth + near_extension) / 2 +
                            (depth_close_crop_offset_ - near_extension);
  marker2.pose.position.y = (right_crop_offset_ - left_crop_offset_) / 2;
  marker2.pose.position.z =
      (crop_height - bottom_lift) / 2 + (bottom_crop_offset_ + bottom_lift);
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = crop_depth + near_extension;
  marker2.scale.y = crop_width;
  marker2.scale.z = crop_height - bottom_lift;
  marker2.color.a = 0.1;
  marker2.color.r = 0.9;
  marker2.color.g = 0.9;
  marker2.color.b = 0.9;
  marker2.lifetime = ros::Duration(0);

  ros::Rate r(1);
  for (int i = 0; i < 5; ++i) {
    if (vis_pub_.getNumSubscribers() > 0) {
      vis_pub_.publish(marker);
      vis_pub_.publish(marker2);
      return;
    }
  }
  
  ROS_WARN("No subscribers to shelf cropping marker.");
}

bool ShelfCropper::cropCallBack(
    pr2_pick_perception::CropShelfRequest &request,
    pr2_pick_perception::CropShelfResponse &response) {
  // Read input cloud.
  sensor_msgs::PointCloud2::ConstPtr kinect_pc_ros =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pc_topic_);
  ros::Time pc_timestamp = kinect_pc_ros->header.stamp;
  std::string cloud_frame_id = kinect_pc_ros->header.frame_id;
  PointCloud<PointXYZRGB>::Ptr kinect_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(*kinect_pc_ros, *kinect_pc);

  // Get the right width and height for the requested bin.
  double cell_width;
  if (request.cellID == "A" || request.cellID == "C" || request.cellID == "D" ||
      request.cellID == "F" || request.cellID == "G" || request.cellID == "I" ||
      request.cellID == "J" || request.cellID == "L") {
    cell_width = cell_width1_;
  } else {
    cell_width = cell_width2_;
  }

  double cell_height;
  if (request.cellID == "A" || request.cellID == "B" || request.cellID == "C" ||
      request.cellID == "J" || request.cellID == "K" || request.cellID == "L") {
    cell_height = cell_height1_;
  } else {
    cell_height = cell_height2_;
  }

  // Transform input data into the frame of the bin.
  bin_frame_id_ = "bin_" + request.cellID;
  tf::StampedTransform cloud_to_bin;
  std::string error_msg;
  if (tf_.canTransform(bin_frame_id_, cloud_frame_id, ros::Time(0),
                       &error_msg)) {
    tf_.lookupTransform(bin_frame_id_, cloud_frame_id, ros::Time(0),
                        cloud_to_bin);
  } else {
    ROS_WARN_THROTTLE(10.0,
                      "The tf from  cloud frame '%s' to bin frame '%s' does "
                      "not seem to be available, will assume it as identity!",
                      cloud_frame_id.c_str(), request.cellID.c_str());
    ROS_WARN("Transform error: %s", error_msg.c_str());
    cloud_to_bin.setIdentity();
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr shelf_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud(*kinect_pc, *shelf_pc, cloud_to_bin);

  // Crop shelf.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cell_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  cell_pc = cropPC(shelf_pc, cell_width, cell_height, depth_cell_, 0);
  visualizeShelf(cell_width, cell_height, depth_cell_);

  sensor_msgs::PointCloud2 cropped_cloud;
  pcl::toROSMsg(*cell_pc, cropped_cloud);
  cropped_cloud.header.frame_id = bin_frame_id_;
  cropped_cloud.header.stamp = pc_timestamp;
  response.cloud = cropped_cloud;

  return true;
}
}  // namespace pr2_pick_perception

int main(int argc, char **argv) {
  ros::init(argc, argv, "shelf_cropper");

  ros::NodeHandle nh;

  pr2_pick_perception::ShelfCropper cropper;

  if (!cropper.initialize()) {
    ROS_FATAL("Shelf cropper initialization failed. Shutting down node.");
    return 1;
  }

  ros::ServiceServer server(nh.advertiseService(
      "shelf_cropper", &pr2_pick_perception::ShelfCropper::cropCallBack,
      &cropper));

  ros::spin();

  return 0;
}
