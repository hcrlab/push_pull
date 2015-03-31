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

  double positions[] = {0.1, 0.2, 0.3};
  bool status;

  for(int i = 0; i < sizeof(positions) / sizeof(double); i++) {
    printf("Setting height to to %0.3f\n", positions[i]);
    status = torso.SetHeight(positions[i]);
    printf("status %d\n", status);
  }

  printf("Setting height to max (%f)\n", Torso::kMaxHeight);
  status = torso.SetHeight(Torso::kMaxHeight);
  printf("status %d\n", status);

  printf("Setting height to min (%f)\n", Torso::kMinHeight);
  status = torso.SetHeight(Torso::kMinHeight);
  printf("status %d\n", status);

  double height = -0.2;
  printf("Setting height outside of range (%0.2f)\n", height);
  status = torso.SetHeight(height);
  printf("status %d\n", status);

  return 0;
}
