# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(multisense)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp laser_assembler ap_msgs adaptive_perception_msgs tf sensor_msgs image_transport image_geometry cv_bridge pcl_ros  )
find_package(PCL REQUIRED)

 find_package(cmake_modules REQUIRED)
 
 FIND_PACKAGE(Eigen REQUIRED)
 
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
link_libraries(${catkin_LIBRARIES} ${PCL_LIBRARIES})

find_package(OpenCV)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
  DEPENDS eigen boost opencv PCL
    CATKIN_DEPENDS roscpp laser_assembler ap_msgs adaptive_perception_msgs tf sensor_msgs image_transport image_geometry cv_bridge pcl_ros 
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

#common commands for building c++ executables and libraries
add_executable(laser_snapshotter src/laser_snapshotter.cpp)
add_executable(save_cloud_topic src/save_cloud_topic.cpp)
add_executable(rgb_pointcloud src/rgb_pc.cpp)
target_link_libraries(rgb_pointcloud ${OpenCV_LIBRARIES})
target_link_libraries(save_cloud_topic ${OpenCV_LIBRARIES})

add_executable(laser_snapshotter_save src/laser_snapshotter_save.cpp)


add_executable(crop_pointcloud src/crop_pc.cpp)
target_link_libraries(crop_pointcloud ${OpenCV_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals regex system thread filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(crop_pointcloud ${Boost_LIBRARIES})

add_executable(crop_image src/crop_image.cpp)
target_link_libraries(crop_image ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(configure_multisense_camera src/configure_camera.cpp)
