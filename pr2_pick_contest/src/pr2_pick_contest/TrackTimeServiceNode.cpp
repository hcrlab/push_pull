// Run this node to start the time service.

#include <ros/ros.h>
#include "pr2_pick_contest/TrackTimeService.h"
using pr2_pick_contest::TrackTimeService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "track_time_service_node");
  TrackTimeService service;
  ros::spin();
  return 0;
}
