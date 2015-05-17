#include <iostream>
#include <dirent.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pr2_pick_perception/ColorHistogram.h"
#include "pr2_pick_perception/DescriptorExample.h"
#include "pr2_pick_perception/ItemDescriptor.h"
#include "pr2_pick_perception/MultiItemCloud.h"
#include "pr2_pick_perception/pcl.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

using std::string;
using std::vector;
using pr2_pick_perception::ColorHistogram;
using pr2_pick_perception::DescriptorExample;
using pr2_pick_perception::ItemDescriptor;
using pr2_pick_perception::MultiItemCloud;
using pr2_pick_perception::ComputeColorHistogram;

void ListFiles(const string& data_dir, vector<string>* files) {
  DIR* dir;
  struct dirent* ent;
  if ((dir = opendir(data_dir.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string name(ent->d_name);
      if (name != "." && name != "..") {
        files->push_back(name);
      }
    }
    closedir(dir);
  } else {
    ROS_ERROR("Could not open data_dir %s", data_dir.c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "convert_rosbag_to_descriptors");
  ros::NodeHandle nh;  // Necessary for ros::Time to work.
  if (argc < 2) {
    std::cout << "rosrun pr2_pick_perception convert_rosbag_to_descriptors "
                 "data_dir output_file" << std::endl;
  }
  std::string data_dir(argv[1]);
  std::string output_file(argv[2]);
  int num_bins = 2;

  // Open output file.
  rosbag::Bag output_bag;
  output_bag.open(output_file, rosbag::bagmode::Write);

  // Get list of input files.
  vector<string> files;
  ListFiles(data_dir, &files);

  for (size_t i = 0; i < files.size(); ++i) {
    const std::string& filename = data_dir + "/" + files[i];
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    vector<string> topics;
    topics.push_back("cell_pc");
    topics.push_back("cropped_cloud");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {
      MultiItemCloud::ConstPtr example = m.instantiate<MultiItemCloud>();
      if (example->labels.size() != 1) {
        continue;
      }
      const sensor_msgs::PointCloud2& ros_cloud = example->cloud;
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(ros_cloud, pcl_cloud);
      vector<int> index;
      pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, index);

      vector<int> histogram;
      ComputeColorHistogram(pcl_cloud, num_bins, &histogram);

      ColorHistogram histogram_msg;
      histogram_msg.num_bins = num_bins;
      histogram_msg.histogram = histogram;

      ItemDescriptor descriptor;
      descriptor.histogram = histogram_msg;

      DescriptorExample desc_example;
      desc_example.descriptor = descriptor;
      desc_example.label = example->labels[0];

      output_bag.write("examples", ros::Time::now(), desc_example);
    }

    bag.close();
  }
  output_bag.close();
  return 0;
}
