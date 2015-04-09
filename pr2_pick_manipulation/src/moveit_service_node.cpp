#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit/move_group_interface/move_group.h"
//#include "pr2_pick_manipulation/arm_navigator.h"
#include "ros/ros.h"
#include "pr2_pick_manipulation/MoveArm.h"

class ArmMover {
public:
  ArmMover ():  service_(n_.advertiseService("moveit_service", &ArmMover::move_arm, this)),
    group_("right_arm")
  {
    // Note : we provide the callback (a member) + a state (this)
    //ls_sub = _n.subscribe("/gazebo/link_states", 10, SuperCoolRobot::link_state_callback, this);
   
  }

  bool move_arm(pr2_pick_manipulation::MoveArm::Request  &req,
         pr2_pick_manipulation::MoveArm::Response &res) {
    group_.setPoseTarget(req.goal);
    res.success = group_.move();
    return true;
    //return true;
  }

  void run() {
    ros::spin();
  }

protected:
  // state here
  ros::NodeHandle n_;
  ros::ServiceServer service_;
  moveit::planning_interface::MoveGroup group_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_service");

  ArmMover arm;
  arm.run();
}
