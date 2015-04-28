#include "pr2_pick_perception/pcl.h"

#include "geometry_msgs/Quaternion.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <gtest/gtest.h>

namespace pr2_pick_perception {
class PclTest : public ::testing::Test {};

void PrintQuaternion(const geometry_msgs::Quaternion& q) {
  std::cout << "(w: " << q.w << ", x: " << q.x << ", y: " << q.y
            << ", z: " << q.z << ")" << std::endl;
}

// Create points along the X axis and verify that planar PCA works.
TEST_F(PclTest, PlanarPcaXAxis) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 12;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (unsigned int i = 0; i < 6; ++i) {
    cloud.points[2 * i].x = i;
    cloud.points[2 * i].y = -1;
    cloud.points[2 * i].z = i;

    cloud.points[2 * i + 1].x = i;
    cloud.points[2 * i + 1].y = 1;
    cloud.points[2 * i + 1].z = i;
  }
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(cloud, ros_cloud);

  geometry_msgs::Quaternion q1;
  geometry_msgs::Quaternion q2;
  double v1;
  double v2;
  PlanarPrincipalComponents(ros_cloud, &q1, &q2, &v1, &v2);
  EXPECT_GT(v1, v2);  // First component should be largest.
  EXPECT_FLOAT_EQ(0.74468085, v1 / (v1 + v2));  // Verified with scipy.
  // First quaternion should be aligned with x-axis.
  EXPECT_FLOAT_EQ(1, q1.w);
  EXPECT_FLOAT_EQ(0, q1.x);
  EXPECT_FLOAT_EQ(0, q1.y);
  EXPECT_FLOAT_EQ(0, q1.z);
  // Second quaternion should be 90 degrees.
  EXPECT_FLOAT_EQ(0.707107, q2.w);
  EXPECT_FLOAT_EQ(0, q2.x);
  EXPECT_FLOAT_EQ(0, q2.y);
  EXPECT_FLOAT_EQ(0.707107, q2.z);
}
}  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pcl_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
