#include "trooper_multisense/configure_camera.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <boost/lexical_cast.hpp>

namespace trooper {

  ConfigureMulitsenseCamera::ConfigureMulitsenseCamera(ros::NodeHandle nh) : notifier_(nh, "ConfigureMulitsenseCamera") {

    // subscribe for auto exposure updates
    sub_auto_exposure_ = nh.subscribe("/multisense_sl/set_auto_exposure", 10,
                                      &ConfigureMulitsenseCamera::setAutoExposure, this);

    // subscribe for light level updates
    sub_light_level_ = nh.subscribe("/multisense_sl/set_light_level", 10,
                                    &ConfigureMulitsenseCamera::setLightLevel, this);
  }

  void ConfigureMulitsenseCamera::setAutoExposure(const std_msgs::Bool::ConstPtr &exposure) {

    // console debug
    ROS_INFO("Setting multisense autoexposure to %s", ((exposure->data)?"true":"false"));

    // build a service request to set the autoexposure
    dynamic_reconfigure::ReconfigureRequest request;
    dynamic_reconfigure::ReconfigureResponse response;

    // turn on or off auto exposure based on the user request
    dynamic_reconfigure::BoolParameter exposure_param;
    exposure_param.name = "auto_exposure";
    exposure_param.value = exposure->data;
    request.config.bools.push_back(exposure_param);
    
    // try to make the service call
    if(!ros::service::call("/multisense_sl/set_parameters", request, response)) 
      notifier_.sendError("Unable to set auto exposure!");
  }

  void ConfigureMulitsenseCamera::setLightLevel(const std_msgs::Float32::ConstPtr &level) {
    // console debug
    ROS_INFO("Setting light level to %f", level->data);

    // validate the light level params, error if invalid
    if((level->data < 0.0) || (level->data > 1.0)) {
      notifier_.sendError("Invalid light level: " + 
                          boost::lexical_cast<std::string>(level->data));
      return;
    }

    // build a service request to set the lighting level
    dynamic_reconfigure::ReconfigureRequest request;
    dynamic_reconfigure::ReconfigureResponse response;

    // set the lighting level
    dynamic_reconfigure::DoubleParameter level_param;
    level_param.name = "led_duty_cycle";
    level_param.value = level->data;
    request.config.doubles.push_back(level_param);

    // turn on or off lighting based on the value
    dynamic_reconfigure::BoolParameter lighting_param;
    lighting_param.name = "lighting";
    lighting_param.value = (level->data != 0.0);
    request.config.bools.push_back(lighting_param);
    
    // try to make the actual config call
    if(!ros::service::call("/multisense_sl/set_parameters", request, response)) 
      notifier_.sendError("Unable to set light level!");
  }

} // namespace trooper

int main(int argc, char **argv) {

  // initialize ROS
  ros::init(argc, argv, "configure_camera");
  ros::NodeHandle nh("~");

  // spawn a camera configuration class, all callbacks are
  // asynchronous ROS callbacks 
  trooper::ConfigureMulitsenseCamera configure(nh);

  // run until terminated
  ros::spin();
  return 0;
}

