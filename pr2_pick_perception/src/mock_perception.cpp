#include <vector>
#include "pr2_pick_perception/mock_perception.h"

namespace pr2_pick_perception {
MockPerception::MockPerception() {
  for (int i=0; i<12; ++i) {
    std::vector<pr2_pick_msgs::Item> v;
    bins_.push_back(v);
  }
}

void MockPerception::SetItems(const int bin_id,
                              const std::vector<pr2_pick_msgs::Item>& items) {
  bins_[bin_id] = items;
}

void MockPerception::GetItems(const int bin_id,
                              std::vector<pr2_pick_msgs::Item>* items) {
  items->clear();
  for (const auto& item : bins_[bin_id]) {
    items->push_back(item);
  }
}
};  // namespace pr2_pick_perception
