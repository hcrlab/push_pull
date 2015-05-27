#include <stdio.h>
#include <ros/ros.h>
#include "pr2_pick_manipulation/driver.h"
#include "pr2_pick_manipulation/driver_service.h"
#include <ros/console.h>
#include <iostream>
namespace pr2_pick_manipulation {
DriverService::DriverService()
    : nh_(),
      angular_server_(nh_.advertiseService("drive_angular_service",
                                          &DriverService::AngularCallback,
                                          this)),
      linear_server_(nh_.advertiseService("drive_linear_service",
                                          &DriverService::LinearCallback,
                                          this)),
      drive_to_pose_server_(nh_.advertiseService("drive_to_pose_service",
                                          &DriverService::PoseCallback,
                                          this)),
      driver_() {
}

bool DriverService::AngularCallback(DriveAngular::Request& request,
                                    DriveAngular::Response& response) {
  bool success = driver_.DriveAngular(request.vel_ccw, request.radians);
  response.success = success;
  return success;
}

bool DriverService::LinearCallback(DriveLinear::Request& request,
                                   DriveLinear::Response& response) {
  bool success = driver_.DriveLinear(request.vel_x, request.vel_y,
                                     request.displacement);
  response.success = success;
  return success;
}

bool DriverService::PoseCallback(DriveToPose::Request& request,
                                   DriveToPose::Response& response) {
  ROS_INFO("Received drive to pose request");
  bool success = driver_.DriveToPose(request.pose, request.linearVelocity,
    request.angularVelocity);
  response.success = success;
  ROS_INFO("Sending response!");
  return success;
}

};  // namespace pr2_pick_manipulation
