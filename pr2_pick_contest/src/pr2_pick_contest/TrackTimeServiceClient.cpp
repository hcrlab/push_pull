#include "ros/ros.h"
#include "pr2_pick_contest/TrackTime.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "track_time_service_client");
	ros::NodeHandle node;

	// Create services to track time
	ros::ServiceClient timeClient = node.serviceClient<pr2_pick_contest::TrackTime>("track_time");

	while(1) {
		pr2_pick_contest::TrackTime srv;
		ros::service::waitForService("track_time");
		timeClient.call(srv);
		ROS_ERROR("%f", srv.response.timeElapsed);

		ros::Duration(1.0).sleep();
	} 
}

