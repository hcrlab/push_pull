cmake_minimum_required(VERSION 2.8.3)
project(obj_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge pcl_ros laser_assembler ap_msgs 
             adaptive_perception_msgs image_transport image_geometry   )
 
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost 1.46.1 REQUIRED COMPONENTS signals regex system thread filesystem)
find_package(OpenCV)
find_package(PCL REQUIRED)

 find_package(cmake_modules REQUIRED)
 
 FIND_PACKAGE(Eigen REQUIRED)

MESSAGE( STATUS "PCL: " ${PCL_LIBRARY_DIRS} )
MESSAGE( STATUS "Boost: "  ${Boost_LIBRARIES} )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
link_libraries(${catkin_LIBRARIES} ${PCL_LIBRARIES})

find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
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
#  INCLUDE_DIRS include
#  LIBRARIES obj_detection
  CATKIN_DEPENDS roscpp cv_bridge pcl_ros laser_assembler ap_msgs 
             adaptive_perception_msgs image_transport image_geometry 
  DEPENDS  eigen boost opencv PCL libqhull libvtk libvtk-qt
)

MESSAGE( STATUS "VTK: " ${VTK_LIBRARIES} )
#add_executable(obj_detection src/detect.cpp)
#target_link_libraries(obj_detection ${VTK_LIBRARIES} log4cxx ${Boost_LIBRARIES})


add_executable(obj_detection_pc src/detectpc.cpp)
target_link_libraries(obj_detection_pc ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})

#add_executable(obj_detection_descr src/detect_descr.cpp)
#target_link_libraries(obj_detection_descr ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}  ##${Boost_LIBRARIES})


#add_executable(obj_detection_sa src/detect_sa.cpp)
#target_link_libraries(obj_detection_sa ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(obj_detection_xtion src/detect_pc_xtion.cpp)
target_link_libraries(obj_detection_xtion ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})

