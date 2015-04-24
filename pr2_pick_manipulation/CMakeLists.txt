cmake_minimum_required(VERSION 2.8.3)
project(pr2_pick_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pr2_common_action_msgs
  pr2_controllers_msgs
  pr2_pick_msgs
  pr2_pick_perception
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
#)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  PushAway.srv
  SetGrippers.srv
  GetGrippers.srv
  GetGripperPositions.srv
  DriveLinear.srv
  DriveAngular.srv
  DriveToPose.srv
  MoveTorso.srv
  TuckArms.srv
  MoveHead.srv
  MoveArm.srv
  GetPose.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    arm_navigator
    driver
    gripper
    torso
    driver_service
    gripper_service
    torso_service
    tuck_arms_service
    move_arm_service
    move_head_service
  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(arm_navigator
  src/arm_navigator.cpp
)
add_library(driver
  src/driver.cpp
)
add_library(gripper
  src/gripper.cpp
)
add_library(torso
  src/torso.cpp
)
add_library(driver_service
  src/driver_service.cpp
)
add_library(gripper_service
  src/gripper_service.cpp
)
add_library(torso_service
  src/torso_service.cpp
)
add_library(tuck_arms_service
  src/tuck_arms_service.cpp
)
add_library(move_head_service
  src/move_head_service.cpp
)
add_library(move_arm_service
  src/move_arm_service.cpp
)

## Declare a cpp executable
add_executable(driver_service_node src/driver_service_node.cpp)
add_executable(gripper_service_node src/gripper_service_node.cpp)
add_executable(torso_service_node src/torso_service_node.cpp)
add_executable(tuck_arms_service_node src/tuck_arms_service_node.cpp)
add_executable(move_head_service_node src/move_head_service_node.cpp)
add_executable(move_arm_service_node src/move_arm_service_node.cpp)
add_executable(moveit_service_node src/moveit_service_node.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(driver_service pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(gripper_service pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(torso_service pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(tuck_arms_service pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(move_head_service pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(driver_service_node pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(gripper_service_node pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(torso_service_node pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(tuck_arms_service_node pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(move_head_service_node pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(move_arm_service_node pr2_pick_manipulation_generate_messages_cpp)
add_dependencies(moveit_service_node pr2_pick_manipulation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(arm_navigator
  ${catkin_LIBRARIES}
)
target_link_libraries(driver
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper
  ${catkin_LIBRARIES}
)
target_link_libraries(torso
  ${catkin_LIBRARIES}
)
target_link_libraries(driver_service
  ${catkin_LIBRARIES}
  driver
)
target_link_libraries(driver_service_node
  ${catkin_LIBRARIES}
  driver_service
)
target_link_libraries(gripper_service
  ${catkin_LIBRARIES}
  gripper
)
target_link_libraries(gripper_service_node
  ${catkin_LIBRARIES}
  gripper_service
)
target_link_libraries(torso_service
  ${catkin_LIBRARIES}
  torso
)
target_link_libraries(torso_service_node
  ${catkin_LIBRARIES}
  torso_service
)
target_link_libraries(tuck_arms_service
  ${catkin_LIBRARIES}
)
target_link_libraries(tuck_arms_service_node
  ${catkin_LIBRARIES}
  tuck_arms_service
)
target_link_libraries(move_head_service
  ${catkin_LIBRARIES}
)
target_link_libraries(move_head_service_node
  ${catkin_LIBRARIES}
  move_head_service
)
target_link_libraries(move_arm_service
  ${catkin_LIBRARIES}
)
target_link_libraries(move_arm_service_node
  ${catkin_LIBRARIES}
  move_arm_service
)
target_link_libraries(moveit_service_node
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pr2_pick_manipulation pr2_pick_manipulation_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pr2_pick_manipulation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
