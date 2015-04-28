#include "pr2_pick_perception/planar_pca_service.h"

#include "geometry_msgs/Quaternion.h"
#include "pr2_pick_perception/PlanarPrincipalComponents.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>

namespace pr2_pick_perception {
PlanarPcaService::PlanarPcaService()
    : nh_(),
      server_(nh_.advertiseService("planar_principal_components",
                                   &PlanarPcaService::Callback, this)) {}

bool PlanarPcaService::Callback(PlanarPrincipalComponents::Request& request,
                                PlanarPrincipalComponents::Response& response) {
  const sensor_msgs::PointCloud2& cloud = request.cluster.pointcloud;

  geometry_msgs::Quaternion component1;
  geometry_msgs::Quaternion component2;
  double value1;
  double value2;
  PlanarPrincipalComponents(cloud, &component1, &component2, &value1, &value2);
  response.first_orientation = component1;
  response.second_orientation = component2;
  response.eigenvalue1 = value1;
  response.eigenvalue2 = value2;
  return true;
}
};  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "planar_pca_service_node");
  pr2_pick_perception::PlanarPcaService service;
  ros::spin();
  return 0;
}
