#include <iostream>

// PCL stuff
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/console/parse.h>

// interactive marker stuff
#include <interactive_markers/interactive_marker_server.h>
#include <math.h>
#include <interactive_markers/menu_handler.h>

// ros stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pr2_grasp_evaluator/TransformPointCloud.h>

namespace vm = visualization_msgs;

namespace point_cloud_transformer {

const double ARROW_DIST_THRESHOLD = 0.05;
const double ARROW_TAIL_RADIUS = 0.01;
const double ARROW_HEAD_RADIUS = 0.02;
const double ARROW_HEAD_LENGTH = 0.05;
const double GHOST_ALPHA = 1.0;
const double TRANSLATION_HANDLE_SCALE = 0.05;
const double CLOUD_SHRINKAGE_FACTOR = 1.0;
const double POINT_SIZE = 0.005;

class PointCloudTransformer
{
public:

  PointCloudTransformer() :
  plane_coefficients(new pcl::ModelCoefficients)
  {
    node_handle = ros::NodeHandle("~");

  }

  bool callback(pr2_grasp_evaluator::TransformPointCloud::Request &req, pr2_grasp_evaluator::TransformPointCloud::Response &resp){

    // load point cloud
    // find planes
    // publish plane markers
    // cluster remaining points into objects
    // publish objects

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    ROS_INFO("Waiting for transform");
    tf_listener.waitForTransform(req.new_frame_id, req.prev_frame_id, ros::Time(0), ros::Duration(10.0));
    ROS_INFO("... done waiting.");
    tf_listener.lookupTransform(req.new_frame_id, req.prev_frame_id, ros::Time(0), transform);
    ROS_INFO("Transforming point cloud...");
    std::cout << transform.getRotation();
    std::cout << transform.getOrigin();
    pcl_ros::transformPointCloud(req.new_frame_id, transform, req.point_cloud, resp.point_cloud);
    return true;
  }
  ros::NodeHandle getNH(){
    return node_handle;
  }

private:

  pcl::ModelCoefficients::Ptr plane_coefficients;
  ros::NodeHandle node_handle;

  inline bool quaternionsEqual(const geometry_msgs::Quaternion &q1, const geometry_msgs::Quaternion &q2) {
    return (q1.x == q2.x) && (q1.y == q2.y) && (q1.z == q2.z) && (q1.w == q2.w);
  }

  
};
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main (int argc, char** argv)
{

  ros::init(argc, argv, "euclidean_segmentation_demo");


  // create PointCloudTransformer
  point_cloud_transformer::PointCloudTransformer PointCloudTransformer;
  ros::NodeHandle node_handle = PointCloudTransformer.getNH();
  ros::ServiceServer service = node_handle.advertiseService("transform_point_cloud", &point_cloud_transformer::PointCloudTransformer::callback, &PointCloudTransformer);

  // start the ROS main loop
  ros::spin();

}