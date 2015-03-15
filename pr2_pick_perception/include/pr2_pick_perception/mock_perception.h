#include <vector>
#include "pr2_pick_msgs/Item.h"

#ifndef _PR2_PICK_PERCEPTION_MOCK_PERCEPTION_H_
#define _PR2_PICK_PERCEPTION_MOCK_PERCEPTION_H_

namespace pr2_pick_perception {
// A mock perception node. Allows you to specify item poses that are found by
// an actual perception implementation, and implements the interface to get
// item poses.
class MockPerception {
 private:
  std::vector<std::vector<pr2_pick_msgs::Item> > bins_;
 public:
  MockPerception();

  // Given a bin ID, returns a list of items found in the bin, including their
  // poses.
  void GetItems(const int bin_id, std::vector<pr2_pick_msgs::Item>* items);

  // Sets the items in the given bin ID.
  void SetItems(const int bin_id,
                const std::vector<pr2_pick_msgs::Item>& items);
};
};  // namespace pr2_pick_perception

#endif
