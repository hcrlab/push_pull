#include <ros/ros.h>
#include <pr2_pick_contest/TrackTime.h>
#ifndef _TRACK_TIME_SERVICE_H_
#define _TRACK_TIME_SERVICE_H_

namespace pr2_pick_contest {
class TrackTimeService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  ros::Time begin_;
  bool Callback(TrackTime::Request& request, TrackTime::Response& response);

 public:
  TrackTimeService();
};
};  // namespace pr2_pick_contest

#endif // _TRACK_TIME_SERVICE_H_
