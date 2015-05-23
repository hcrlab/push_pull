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
  pcl_conversions
  pcl_ros
  pr2_pick_msgs
  roscpp
  sensor_msgs
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# -std=c++0x"
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
  BoundingBox.msg
  Box.msg
  Cell.msg
  Cluster.msg
  ClusterList.msg
  ColorHistogram.msg
  ItemDescriptor.msg
  Object.msg
  ObjectDetectionRequest.msg
  ObjectList.msg
  MultiItemCloud.msg
  ROI2d.msg
  DescriptorExample.msg
  ObjectMatchRequest.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  BoxPoints.srv
  CropShelf.srv
  DeleteStaticTransform.srv
  FindCentroid.srv
  LocalizeShelf.srv
  SetStaticTransform.srv
  PlanarPrincipalComponents.srv
  GetItemDescriptor.srv
  ClassifyTargetItem.srv
  CountPointsInBox.srv
  SegmentItems.srv
  MatchCluster.srv
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

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if you package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    actionlib_msgs
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    laser_assembler
    pcl_conversions
    pcl_ros
    pr2_pick_msgs
    roscpp
    sensor_msgs
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
add_library(pcl_lib
  src/pcl.cpp
)

## Declare a cpp executable
add_executable(shelf_localization_service_node
  src/shelf_localization.cpp
)

add_executable(shelf_localization_service_node_lidar
  src/shelf_localization_LIDAR.cpp
)

add_executable(object_recognition_service
  src/obj_recognition_cluster.cpp
)
#add_executable(test_match_descriptor
#  src/test_matcher.cpp
#)

add_executable(realsense_camerainfo
  src/camera_info.cpp
)

add_executable(pointcloud_realsense
  src/pointcloud_realsense.cpp
)


add_executable(shelf_cropping_service_node
  src/shelf_cropping.cpp
)

add_executable(item_segmentation_service_node
  src/item_segmentation.cpp
)

add_executable(planar_pca_service_node
  src/planar_pca_service.cpp
)

add_executable(item_descriptor_service_node
  src/item_descriptor_service.cpp
)

add_executable(count_points_in_box_service_node
  src/count_points_in_box_service.cpp
)

add_executable(cluster_points_in_box_service_node
  src/cluster_points_in_box_service.cpp
)

add_executable(convert_rosbag_to_descriptors
  src/convert_rosbag_to_descriptors.cpp
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(shelf_localization_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(shelf_localization_service_node_lidar
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(shelf_cropping_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(item_segmentation_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(planar_pca_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(item_descriptor_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(count_points_in_box_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(cluster_points_in_box_service_node
  pr2_pick_msgs_generate_messages_cpp
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(convert_rosbag_to_descriptors
  pr2_pick_perception_generate_messages_cpp
)

add_dependencies(object_recognition_service
  pr2_pick_perception_generate_messages_cpp
)

# Hopefully not needed.
#add_dependencies(pcl_lib
#  pr2_pick_msgs_generate_messages_cpp
#  pr2_pick_perception_generate_messages_cpp
#)

## Specify libraries to link a library or executable target against
target_link_libraries(shelf_localization_service_node
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(pointcloud_realsense
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)



target_link_libraries(realsense_camerainfo
  ${catkin_LIBRARIES}
)


# target_link_libraries(shelf_localization_service_node_lidar
#   ${VTK_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

target_link_libraries(shelf_cropping_service_node
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#target_link_libraries(test_match_descriptor
#  ${VTK_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(object_recognition_service
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


target_link_libraries(item_segmentation_service_node
  ${VTK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  pcl_lib
)

target_link_libraries(pcl_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(planar_pca_service_node
  ${catkin_LIBRARIES}
  pcl_lib
)
target_link_libraries(item_descriptor_service_node
  ${catkin_LIBRARIES}
  pcl_lib
)
target_link_libraries(count_points_in_box_service_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cluster_points_in_box_service_node
  ${catkin_LIBRARIES}
)
target_link_libraries(convert_rosbag_to_descriptors
  ${catkin_LIBRARIES}
  pcl_lib
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(pcl_test src/pcl_test.cpp)
if(TARGET pcl_test)
  target_link_libraries(pcl_test pcl_lib)
endif()
