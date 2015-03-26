#include <ros/ros.h>

#include "pr2_pick_manipulation/torso.h"

using pr2_pick_manipulation::Torso;

/**
 * A simple demonstration of the torso client declared in
 * pr2_pick_manipulation/torso.h
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "torso_demo");
  ros::NodeHandle node_handle;

  Torso torso;

  printf("Setting height to 0.1\n");
  torso.SetHeight(0.1);

  printf("Setting height to max (%f)\n", Torso::kMaxHeight);
  torso.SetHeight(Torso::kMaxHeight);

  printf("Setting height to min (%f)\n", Torso::kMinHeight);
  torso.SetHeight(Torso::kMinHeight);

  return 0;
}
