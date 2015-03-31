#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <trooper_common/trooper_notification_broadcaster.h>

namespace trooper {

  class ConfigureMulitsenseCamera {

    public:

      /// Default constructor, spawn subscribers for changes from OCU.
      ConfigureMulitsenseCamera(ros::NodeHandle nh);

      /// Callback to set auto exposure.
      void setAutoExposure(const std_msgs::Bool::ConstPtr &exposure);

      /// Callback to set light level.
      void setLightLevel(const std_msgs::Float32::ConstPtr &level);

    private:

      /// Subscriber to turn on/off auto exposure.
      ros::Subscriber sub_auto_exposure_;

      /// Subscriber for light level 0.0 - 1.0 (where 0 is off and 1 is full).
      ros::Subscriber sub_light_level_;

      /// Notify the user if there is an error in setting a value.
      NotificationBroadcaster notifier_;

  }; // ConfigureMulitsenseCamera

} // namespace trooper

