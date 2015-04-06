#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "pr2_pick_manipulation/arm_navigator.h"
#include "pr2_pick_manipulation/driver.h"
#include "pr2_pick_manipulation/gripper.h"
#include "pr2_pick_msgs/Item.h"
#include "pr2_pick_perception/GetItemsAction.h"
#include "pr2_pick_perception/GetItemsGoal.h"
#include "std_msgs/String.h"
#include "moveit/move_group_interface/move_group.h"

static const std::string ROBOT_DESCRIPTION="robot_description";

using actionlib::SimpleActionClient;
using geometry_msgs::PoseStamped;
using pr2_pick_msgs::Item;
using pr2_pick_manipulation::Gripper;
using pr2_pick_manipulation::ArmNavigatorInterface;
using ros::Publisher;

bool MoveToPoseGoal(ArmNavigatorInterface& navigator, const PoseStamped& pose,
                    const bool refresh_point_cloud);
bool Pick(ArmNavigatorInterface& navigator, Gripper& gripper, const Item& item);
bool DropOff(ArmNavigatorInterface& navigator, Gripper& gripper);
bool CenterArm(ArmNavigatorInterface& group);
void Say(std::string text);

ros::Publisher pub;

void Say(std::string text) {
  std_msgs::String str;
  str.data = text;
  pub.publish(str);
}

bool MoveToPoseGoal(ArmNavigatorInterface& navigator, const PoseStamped& pose,
                    const bool refresh_point_cloud) {
  return navigator.MoveToPoseGoal(pose, refresh_point_cloud);
}

bool CenterArm(ArmNavigatorInterface& navigator) {
  PoseStamped pose;
  pose.header.frame_id = "base_footprint";
  pose.pose.position.x = 0.3135;
  pose.pose.position.y = -0.4665;
  pose.pose.position.z = 0.6905;
  pose.pose.orientation.x = -0.7969;
  pose.pose.orientation.y = 0.2719;
  pose.pose.orientation.z = -0.4802;
  pose.pose.orientation.w = -0.2458;
  //pose.pose.position.x = 0.1;
  //pose.pose.position.y = -0.6637;
  //pose.pose.position.z = 0.541;
  //pose.pose.orientation.x = 1;
  //pose.pose.orientation.y = 0;
  //pose.pose.orientation.z = 0;
  //pose.pose.orientation.w = 0;

  bool success = false;
  Say("Centering arm");
  success = navigator.MoveToPoseGoal(pose, false);
  if (!success) {
    ROS_ERROR("Failed to move to reset position.");
    Say("Failed to move to reset position.");
    return false;
  }
  return true;
}

bool DropOff(ArmNavigatorInterface& navigator, Gripper& gripper) {
  PoseStamped pose;
  pose.header.frame_id = "base_footprint";
  pose.pose.position.x = 0.0806;
  pose.pose.position.y = -0.6637;
  pose.pose.position.z = 0.3341;
  pose.pose.orientation.x = 0.5252;
  pose.pose.orientation.y = -0.4382;
  pose.pose.orientation.z = -0.4657;
  pose.pose.orientation.w = -0.5615;
  bool success = false;
  Say("Dropping off");
  success = MoveToPoseGoal(navigator, pose, false);
  if (!success) {
    ROS_ERROR("Failed to move to drop off location.");
    Say("Failed to move to drop off location.");
    return false;
  }
  success = gripper.open();
  return success;
}

// Mock picking method. Assumes that object is graspable from the front, with
// the gripper opening horizontally.
// Assumes that the item's position refers to the center of the item.
// Assumes that items are approximately 10 cm long in the x dimension.
bool Pick(ArmNavigatorInterface& navigator, Gripper& gripper, const Item& item) {
  PoseStamped pose = item.pose;

  bool success = false;
  // Open gripper.
  gripper.open();  

  // Move to a pose 10 cm in front of the item.
  PoseStamped goal = pose;
  goal.pose.position.x = item.pose.pose.position.x - 0.15;
  ROS_INFO("Pose goal x: %f", goal.pose.position.x);
  ROS_INFO("Pose goal y: %f", goal.pose.position.y);
  ROS_INFO("Pose goal z: %f", goal.pose.position.z);

  Say("Rough moving...");
  success = MoveToPoseGoal(navigator, goal, true);
  if (!success) {
    ROS_ERROR("Failed to rough move in front of item.");
    Say("Failed to rough move in front of item.");
    return false;
  }
  
  // Move to a pose 5 cm from the item.
  goal.pose.position.x = item.pose.pose.position.x - 0.05;
  Say("Precise moving...");
  success = MoveToPoseGoal(navigator, goal, false);
  if (!success) {
    ROS_ERROR("Failed to precise move in front of item.");
    Say("Failed to precise move in front of item.");
    return false;
  }
  
  // Close gripper
  gripper.close();

  // Move to a pose 5 cm up.
  goal.pose.position.z = item.pose.pose.position.z + 0.04;
  Say("Lifting item up");
  success = MoveToPoseGoal(navigator, goal, false);
  if (!success) {
    ROS_ERROR("Failed to move post-grasp.");
    Say("Failed to move post-grasp.");
    return false;
  }

  // Move 10 cm out.
  goal.pose.position.x = item.pose.pose.position.x - 0.10;
  Say("Retreating item");
  success = MoveToPoseGoal(navigator, goal, false);
  if (!success) {
    ROS_ERROR("Failed to move post-grasp.");
    Say("Failed to move post-grasp.");
    return false;
  }
  
  // Move 20 cm out.
  goal.pose.position.x = item.pose.pose.position.x - 0.25;
  Say("Removing item fully");
  success = MoveToPoseGoal(navigator, goal, false);
  if (!success) {
    ROS_ERROR("Failed to fully remove item.");
    Say("Failed to fully remove item.");
    return false;
  }
  
  return true;
}

int main(int argc, char** argv) {
  // Set up ROS node.
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //ArmNavigatorInterface navigator(node_handle, pr2_pick_manipulation::kRight);
  // Set up MoveIt.
  moveit::planning_interface::MoveGroup group("right_arm");
  pr2_pick_manipulation::ArmNavigatorInterface* navigator = new pr2_pick_manipulation::MoveItArmNavigator(group);
  //group.setGoalPositionTolerance(0.05);
  //group.setGoalOrientationTolerance(0.2);

  Gripper right_gripper("r_gripper_controller/gripper_action");

  // Set up mock perception.
  SimpleActionClient<pr2_pick_perception::GetItemsAction> mock_perception(
    "mock_perception_action_server", true);  
  mock_perception.waitForServer();

  pr2_pick_manipulation::RobotDriver driver;
  pub = node_handle.advertise<std_msgs::String>("festival_tts", 10);

  // Grab the first 2 objects on the right.
  for (int i=0; i<3; ++i) {
    double distance = 0.2921 * i;
    Say("Driving to the left");
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.y = 0.125;
    driver.Drive(base_cmd, distance);

    pr2_pick_perception::GetItemsGoal goal;
    goal.bin_id = 6 + i;
    mock_perception.sendGoal(goal);
    mock_perception.waitForResult(ros::Duration(30));
    boost::shared_ptr<const pr2_pick_perception::GetItemsResult> result = mock_perception.getResult();
    pr2_pick_msgs::Item item = result->items[0];

    bool success = false;
    success = Pick(*navigator, right_gripper, item);
    if (!success) {
      ROS_ERROR("Failed to pick item.");
      Say("Failed to pick item.");
      CenterArm(*navigator);
      ROS_ERROR("Driving to the right");
      Say("Driving to the right");
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
      base_cmd.linear.y = -0.25;
      driver.Drive(base_cmd, distance);
      break;
    }

    Say("Driving to the right");
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.y = -0.125;
    driver.Drive(base_cmd, distance);

    success = DropOff(*navigator, right_gripper);
    if (!success) {
      ROS_ERROR("Failed to drop off item.");
      Say("Failed to drop off item.");
      break;
    }

    CenterArm(*navigator);
  }

  right_gripper.open();

  sleep(5.0);
  Say("Shutting down demo.");

  ros::shutdown();
  return 0;
}
