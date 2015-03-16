#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit_msgs/AttachedCollisionObject.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "pr2_pick_manipulation/gripper.h"
#include "pr2_pick_msgs/Item.h"
#include "pr2_pick_perception/GetItemsAction.h"
#include "pr2_pick_perception/GetItemsGoal.h"
#include "shape_tools/solid_primitive_dims.h"

static const std::string ROBOT_DESCRIPTION="robot_description";

using geometry_msgs::PoseStamped;
using moveit::planning_interface::MoveGroup;
using moveit::planning_interface::PlanningSceneInterface;
using moveit_msgs::CollisionObject;
using moveit_msgs::DisplayTrajectory;
using pr2_pick_msgs::Item;
using pr2_pick_manipulation::Gripper;
using ros::Publisher;

bool MoveToPoseGoal(MoveGroup& group, const PoseStamped& pose);
bool Pick(MoveGroup& group, Gripper& gripper, const Item& item);
bool DropOff(MoveGroup& group, Gripper& gripper);
bool CenterArm(MoveGroup& group);

bool MoveToPoseGoal(MoveGroup& group, const PoseStamped& pose) {
  group.setPoseTarget(pose);
  //MoveGroup::Plan plan;
  //group.plan(plan);
  //ROS_INFO("Visualizing plan.");
  //sleep(5);
  bool success = group.move();
  return success;
}

bool CenterArm(MoveGroup& group) {
  PoseStamped pose;
  pose.header.frame_id = "base_footprint";
  pose.pose.position.x = 0.3135;
  pose.pose.position.y = -0.4665;
  pose.pose.position.z = 0.7905;
  pose.pose.orientation.x = -0.7969;
  pose.pose.orientation.y = 0.2719;
  pose.pose.orientation.z = -0.4802;
  pose.pose.orientation.w = -0.2458;
  bool success = false;
  success = MoveToPoseGoal(group, pose);
  if (!success) {
    ROS_ERROR("Failed to move to reset position.");
    return false;
  }
  return true;
}

bool DropOff(MoveGroup& group, Gripper& gripper) {
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
  success = MoveToPoseGoal(group, pose);
  if (!success) {
    ROS_ERROR("Failed to move to drop off location.");
    return false;
  }
  success = gripper.open();
  return success;
}

// Mock picking method. Assumes that object is graspable from the front, with
// the gripper opening horizontally.
// Assumes that the item's position refers to the center of the item.
// Assumes that items are approximately 10 cm long in the x dimension.
bool Pick(MoveGroup& group, Gripper& gripper, const Item& item) {
  PoseStamped pose = item.pose;
  MoveGroup::Plan plan;

  bool success = false;
  // Open gripper.
  gripper.open();  

  // Move to a pose 10 cm in front of the item.
  PoseStamped goal = pose;
  goal.pose.position.x = item.pose.pose.position.x - 0.1;
  success = MoveToPoseGoal(group, goal);
  if (!success) {
    ROS_ERROR("Failed to rough move in front of item.");
    return false;
  }
  
  // Move to a pose 5 cm from the item.
  goal.pose.position.x = item.pose.pose.position.x - 0.05;
  success = MoveToPoseGoal(group, goal);
  if (!success) {
    ROS_ERROR("Failed to precise move in front of item.");
    return false;
  }
  
  // Close gripper
  gripper.close();

  // Move to a pose 5 cm up.
  goal.pose.position.z = item.pose.pose.position.z + 0.05;
  success = MoveToPoseGoal(group, goal);
  if (!success) {
    ROS_ERROR("Failed to move post-grasp.");
    return false;
  }

  // Move 10 cm out.
  goal.pose.position.x = item.pose.pose.position.x - 0.15;
  success = MoveToPoseGoal(group, goal);
  if (!success) {
    ROS_ERROR("Failed to move post-grasp.");
    return false;
  }
  
  // Move 25 cm out.
  goal.pose.position.x = item.pose.pose.position.x - 0.3;
  success = MoveToPoseGoal(group, goal);
  if (!success) {
    ROS_ERROR("Failed to fully remove item.");
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

  // Set up MoveIt.
  MoveGroup right_group("right_arm");
  MoveGroup left_group("left_arm");
  moveit::planning_interface::MoveGroup::Plan my_plan;
  ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1, true
    );
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  right_group.allowReplanning(true);
  left_group.allowReplanning(true);

  Gripper right_gripper("r_gripper_controller/gripper_action");
  Gripper left_gripper("r_gripper_controller/gripper_action");

  // Set up mock perception.
  actionlib::SimpleActionClient<pr2_pick_perception::GetItemsAction> ac(
    "mock_perception_action_server", true);  
  ac.waitForServer();

  for (int i=8; i>7; --i) {
    pr2_pick_perception::GetItemsGoal goal;
    goal.bin_id = i;
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(30));
    boost::shared_ptr<const pr2_pick_perception::GetItemsResult> result = ac.getResult();
    pr2_pick_msgs::Item item = result->items[0];
    ROS_WARN("Item pose x: %f", item.pose.pose.position.x);
    ROS_WARN("Item pose y: %f", item.pose.pose.position.y);
    ROS_WARN("Item pose z: %f", item.pose.pose.position.z);

    bool success = false;
    success = Pick(right_group, right_gripper, item);
    if (!success) {
      ROS_ERROR("Failed to pick item.");
      break;
    }
    success = DropOff(right_group, right_gripper);
    if (!success) {
      ROS_ERROR("Failed to drop off item.");
    }

    CenterArm(right_group);
  }
  right_gripper.open();
  left_gripper.open();

  sleep(10.0);
  ROS_INFO("Shutting down demo.");

  ros::shutdown();
  return 0;
}
