#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "pr2_pick_msgs/Item.h"
#include "pr2_pick_perception/mock_perception.h"
#include "pr2_pick_perception/GetItemsAction.h"

using pr2_pick_perception::MockPerception;
using pr2_pick_msgs::Item;

// MockPerception action server.
// TODO(jstn): Generalize this to all kinds of Perception interfaces.
class EasyBoxesAction {
 protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pr2_pick_perception::GetItemsAction> as_; 
  MockPerception mock_perception_;

 public:
  EasyBoxesAction(const std::string& name,
                  const MockPerception& mock_perception):
      nh_(name),
      as_(nh_, name, boost::bind(&EasyBoxesAction::executeCB, this, _1),
          false),
      mock_perception_(mock_perception) {
    as_.start();
  }

  ~EasyBoxesAction() {
  }

  void executeCB(const pr2_pick_perception::GetItemsGoalConstPtr &goal) {
    std::vector<Item> items;
    mock_perception_.GetItems(goal->bin_id, &items);
    pr2_pick_perception::GetItemsResult result;
    result.items = items;
    as_.setSucceeded(result);
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "mock_perception_action");
  MockPerception mock_perception;
  Item index_card;
  index_card.id = "mead_index_cards";
  index_card.pose.header.frame_id="base_footprint";
  index_card.pose.pose.position.x = 0.423;
  index_card.pose.pose.position.y = -0.116;
  index_card.pose.pose.position.z = 1.157;
  mock_perception.SetItems(6, {index_card});
  index_card.pose.pose.position.x = 0.465;
  index_card.pose.pose.position.y = -0.142;
  index_card.pose.pose.position.z = 0.998;
  mock_perception.SetItems(7, {index_card});
  index_card.pose.pose.position.x = 0.496;
  index_card.pose.pose.position.y = -0.378;
  index_card.pose.pose.position.z = 0.977;
  mock_perception.SetItems(8, {index_card});
  EasyBoxesAction m(ros::this_node::getName(), mock_perception);
  ros::spin();
  return 0;
}
