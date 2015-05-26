#include <ros/ros.h>
#include "pr2_pick_contest/TrackTimeService.h"

namespace pr2_pick_contest {
TrackTimeService::TrackTimeService()
  : nh_(),
		begin_(ros::Time::now().toSec()),
    server_(nh_.advertiseService("track_time",
                                 &TrackTimeService::Callback,
                                 this)){
}

bool TrackTimeService::Callback(TrackTime::Request& request,
                              TrackTime::Response& response) {

	response.timeElapsed = (ros::Time::now() - begin_).toSec();
	return true;

}
};  // namespace pr2_pick_contest
