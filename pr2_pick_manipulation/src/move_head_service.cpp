#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pr2_pick_manipulation/move_head_service.h"

using actionlib::SimpleClientGoalState;

namespace pr2_pick_manipulation {
	MoveHeadService::MoveHeadService()
	  : nh_(),
	  server_(nh_.advertiseService("move_head_service",
	                               &MoveHeadService::Callback,
	                               this)),
  	client_("/head_traj_controller/point_head_action", true) {
	}

	bool MoveHeadService::Callback(MoveHead::Request& request,
	                               MoveHead::Response& response) {
	  ROS_INFO("Waiting for move head action server...");
	  client_.waitForServer();
	  ROS_INFO("Move head service ready.");
	  //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = request.frame;
    point.point.x = request.x;
    point.point.y = request.y;
    point.point.z = request.z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    client_.sendGoal(goal);

    bool success = false;
    //wait for it to get there (abort after 30 secs to prevent getting stuck)
    success = client_.waitForResult(ros::Duration(30.0));

    if (success) {
	    SimpleClientGoalState state = client_.getState();
	    if (state != SimpleClientGoalState::SUCCEEDED) {
	      ROS_WARN("Move head action ended in state: %s", state.toString().c_str());
	    }
	    response.success = true;
	    return true;
	  } else {
	    response.success = false;
	    return false;
	  }
	}
};  // namespace pr2_pick_manipulation
