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
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
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

  geometry_msgs::Quaternion q1;
  geometry_msgs::Quaternion q2;
  double v1;
  double v2;
  PlanarPrincipalComponents(cloud, &q1, &q2, &v1, &v2);
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

// Create points along the X axis, but not centered.
TEST_F(PclTest, PlanarPcaXAxisShifted) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 12;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (unsigned int i = 0; i < 6; ++i) {
    cloud.points[2 * i].x = i + 10;
    cloud.points[2 * i].y = 39;
    cloud.points[2 * i].z = i;

    cloud.points[2 * i + 1].x = i + 10;
    cloud.points[2 * i + 1].y = 41;
    cloud.points[2 * i + 1].z = i;
  }

  geometry_msgs::Quaternion q1;
  geometry_msgs::Quaternion q2;
  double v1;
  double v2;
  PlanarPrincipalComponents(cloud, &q1, &q2, &v1, &v2);
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

// Test ComputeColorHistogram with a black point.
TEST_F(PclTest, ComputeColorHistogramAll0s) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.points[0].r = 0;
  cloud.points[0].g = 0;
  cloud.points[0].b = 0;

  std::vector<int> histogram;
  ComputeColorHistogram(cloud, 4, &histogram);
  for (size_t i = 0; i < histogram.size(); ++i) {
    if (i % 4 == 0) {
      EXPECT_EQ(1, histogram[i]);
    } else {
      EXPECT_EQ(0, histogram[i]);
    }
  }
}

// Test ComputeColorHistogram with colors near the upper and lower bounds of a
// bin.
TEST_F(PclTest, ComputeColorHistogramBoundaries) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 2;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.points[0].r = 63;   // Bin 0
  cloud.points[0].g = 64;   // Bin 1
  cloud.points[0].b = 127;  // Bin 1
  cloud.points[1].r = 128;  // Bin 2
  cloud.points[1].g = 191;  // Bin 2
  cloud.points[1].b = 192;  // Bin 3

  std::vector<int> histogram;
  ComputeColorHistogram(cloud, 4, &histogram);

  // Expect red 0, red 2.
  EXPECT_EQ(1, histogram[0]);
  EXPECT_EQ(0, histogram[1]);
  EXPECT_EQ(1, histogram[2]);
  EXPECT_EQ(0, histogram[3]);

  // Expect green 1, green 2.
  EXPECT_EQ(0, histogram[4]);
  EXPECT_EQ(1, histogram[5]);
  EXPECT_EQ(1, histogram[6]);
  EXPECT_EQ(0, histogram[7]);

  // Expect blue 1, blue 3.
  EXPECT_EQ(0, histogram[8]);
  EXPECT_EQ(1, histogram[9]);
  EXPECT_EQ(0, histogram[10]);
  EXPECT_EQ(1, histogram[11]);
}

// Test ComputeColorHistogram with a white point.
TEST_F(PclTest, ComputeColorHistogramAll255s) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.points[0].r = 255;
  cloud.points[0].g = 255;
  cloud.points[0].b = 255;

  std::vector<int> histogram;
  ComputeColorHistogram(cloud, 4, &histogram);
  for (size_t i = 0; i < histogram.size(); ++i) {
    if (i % 4 == 3) {
      EXPECT_EQ(1, histogram[i]);
    } else {
      EXPECT_EQ(0, histogram[i]);
    }
  }
}

// Test ComputeColorHistogram with a white pixel with 5 bins. If each bin is of
// size 255 / 5 = 51, then a pixel value of 255 would be put into bin 255 / 51 =
// 5, which is out of bounds. Check that the boundary detection works.
TEST_F(PclTest, ComputeColorHistogram5Bins) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.points[0].r = 255;
  cloud.points[0].g = 255;
  cloud.points[0].b = 255;

  std::vector<int> histogram;
  ComputeColorHistogram(cloud, 5, &histogram);
  for (size_t i = 0; i < histogram.size(); ++i) {
    if (i % 5 == 4) {
      EXPECT_EQ(1, histogram[i]);
    } else {
      EXPECT_EQ(0, histogram[i]);
    }
  }
}

// Create points going up diagonally in the XZ plane.
TEST_F(PclTest, MinimumBoundingBoxDiagonal) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 12;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (unsigned int i = 0; i < 6; ++i) {
    cloud.points[2 * i].x = i;
    cloud.points[2 * i].y = 59;
    cloud.points[2 * i].z = i;

    cloud.points[2 * i + 1].x = i;
    cloud.points[2 * i + 1].y = 61;
    cloud.points[2 * i + 1].z = i;
  }

  geometry_msgs::Pose centroid;
  geometry_msgs::Vector3 dimensions;
  MinimumBoundingBox(cloud, &centroid, &dimensions);

  // Centroid is at 3.5, 60, 3.5.
  // Orientation is pi, pi/4, pi (roll, pitch, yaw).
  // Translation to/from quaternion verified using tf.transformations.
  // Dimensions are sqrt(50) in the principal component, 2 in the 2nd, and 0 for
  // the last.
  EXPECT_FLOAT_EQ(2.5, centroid.position.x);
  EXPECT_FLOAT_EQ(60, centroid.position.y);
  EXPECT_FLOAT_EQ(2.5, centroid.position.z);
  EXPECT_FLOAT_EQ(0, centroid.orientation.x);
  EXPECT_FLOAT_EQ(0.92387956, centroid.orientation.y);
  EXPECT_FLOAT_EQ(0, centroid.orientation.z);
  EXPECT_FLOAT_EQ(0.3826834, centroid.orientation.w);
  EXPECT_FLOAT_EQ(7.07106781187, dimensions.x);
  EXPECT_FLOAT_EQ(2, dimensions.y);
  EXPECT_FLOAT_EQ(0, dimensions.z);
}

// Create a 2x4 box that is longest in the x direction and 2nd longest in the z
// direction..
TEST_F(PclTest, MinimumBoundingBoxRotateAxis) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = 8;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  double points[8][3] = {{0, 1, 0},
                         {0, 1, 4},
                         {0, -1, 0},
                         {0, -1, 4},
                         {12, 1, 0},
                         {12, 1, 4},
                         {12, -1, 0},
                         {12, -1, 4}};
  for (unsigned int i = 0; i < 8; ++i) {
    cloud.points[i].x = points[i][0];
    cloud.points[i].y = points[i][1];
    cloud.points[i].z = points[i][2];
  }

  geometry_msgs::Pose centroid;
  geometry_msgs::Vector3 dimensions;
  MinimumBoundingBox(cloud, &centroid, &dimensions);

  // Centroid is at 6, 0, 2
  // Orientation is pi/2, 0, 0 (roll, pitch, yaw).
  // Translation to/from quaternion verified using tf.transformations.
  // Dimensions are 12, 4, and 2.
  EXPECT_FLOAT_EQ(6, centroid.position.x);
  EXPECT_FLOAT_EQ(0, centroid.position.y);
  EXPECT_FLOAT_EQ(2, centroid.position.z);
  EXPECT_FLOAT_EQ(0.70710678, centroid.orientation.x);
  EXPECT_FLOAT_EQ(0, centroid.orientation.y);
  EXPECT_FLOAT_EQ(0, centroid.orientation.z);
  EXPECT_FLOAT_EQ(0.70710678, centroid.orientation.w);
  EXPECT_FLOAT_EQ(12, dimensions.x);
  EXPECT_FLOAT_EQ(4, dimensions.y);
  EXPECT_FLOAT_EQ(2, dimensions.z);
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
