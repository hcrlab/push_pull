#include "pr2_pick_manipulation/MoveTorso.h"
#include "pr2_pick_manipulation/torso.h"
#include "pr2_pick_manipulation/torso_service.h"

#include <ros/ros.h>

namespace pr2_pick_manipulation {
TorsoService::TorsoService()
    : nh_(),
      server_(nh_.advertiseService("torso_service",
                                   &TorsoService::Callback,
                                   this)),
      torso_() {
}

bool TorsoService::Callback(MoveTorso::Request& request,
                            MoveTorso::Response& response) {
  ROS_INFO("Received torso move request");

  if (request.height < Torso::kMinHeight) {
    ROS_WARN("Torso service: requested height %f was lower than the minimum of"
             " %f", request.height, Torso::kMinHeight);
    request.height = Torso::kMinHeight;
  }
  if (request.height > Torso::kMaxHeight) {
    ROS_WARN("Torso service: requested height %f was taller than the maximum of"
             " %f", request.height, Torso::kMaxHeight);
    request.height = Torso::kMaxHeight;
  }
  bool success = torso_.SetHeight(request.height, request.blocking);
  ROS_INFO("Returning torso move response");
  response.success = success;
  
  if (!request.blocking) {
    response.success = true;
  }

  return response.success;
}

};  // namespace pr2_pick_manipulation
