cmake_minimum_required(VERSION 2.8.3)
project(pr2_pick_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  laser_assembler
  message_generation
  pcl_ros
  pr2_pick_msgs
  roscpp
  tf
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Boost 1.46.1 REQUIRED COMPONENTS signals regex system thread filesystem)
find_package(OpenCV)
find_package(PCL REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

MESSAGE( STATUS "PCL: " ${PCL_LIBRARY_DIRS} )
MESSAGE( STATUS "Boost: "  ${Boost_LIBRARIES} )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
link_libraries(${catkin_LIBRARIES} ${PCL_LIBRARIES})

find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

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
add_message_files(
  FILES
  Object.msg
  ObjectList.msg
  ObjectDetectionRequest.msg
  ROI2d.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LocalizeShelf.srv
  SetStaticTransform.srv
  DeleteStaticTransform.srv
   CropShelf.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GetItems.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  pr2_pick_msgs
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
  LIBRARIES mock_perception
  CATKIN_DEPENDS
    actionlib_msgs
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    laser_assembler
    pcl_ros
    pr2_pick_msgs
    roscpp
  DEPENDS
    eigen
    boost
    opencv
    PCL
    libqhull
    libvtk
    libvtk-qt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

## Declare a cpp library
add_library(mock_perception
  src/mock_perception.cpp
)
add_executable(shelf_localization_service_node
  src/shelf_localization.cpp
)

add_executable(shelf_cropping_service_node
  src/shelf_cropping.cpp
)


## Declare a cpp executable
add_executable(mock_perception_easy_boxes src/mock_perception_easy_boxes.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(mock_perception
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)
add_dependencies(shelf_localization_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(shelf_cropping_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
#target_link_libraries(mock_perception ${catkin_LIBRARIES})
#target_link_libraries(mock_perception_easy_boxes
#  mock_perception
#  ${catkin_LIBRARIES}
#)
target_link_libraries(shelf_localization_service_node
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}

)
target_link_libraries(shelf_cropping_service_node
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
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
# install(TARGETS perception_tools perception_tools_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_perception_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
