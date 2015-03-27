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

  double zero = 0.000000000000000;
  if (zero < 0) {
    printf("float error. variable zero is %f\n", zero);
    return 1;
  }

  // Why does this hang? What's the actual lower limit?
  // printf("Setting height to 0.0\n");
  // torso.SetHeight(zero);

  printf("Setting height to 0.2\n");
  torso.SetHeight(0.2);

  printf("Setting height to 0.3\n");
  torso.SetHeight(0.3);

  printf("Setting height to 0.33\n");
  torso.SetHeight(0.330000);

  printf("Setting height to max (%f)\n", Torso::kMaxHeight);
  torso.SetHeight(Torso::kMaxHeight);

  printf("Setting height to min (%f)\n", Torso::kMinHeight);
  torso.SetHeight(Torso::kMinHeight);

  return 0;
}
