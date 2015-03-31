# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ap_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs actionlib_msgs moveit_msgs trajectory_msgs message_generation)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#uncomment if you have defined messages
add_message_files(
  FILES
    ObjectList.msg
    Object.msg
#    WorldModelOperation.msg    
 #   WorldModelAuthor.msg
  #  Sphere.msg
   # Cylinder.msg
   # Box.msg
)
#uncomment if you have defined services
add_service_files(
  FILES
  )
  
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs  
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
  #DEPENDS 
    CATKIN_DEPENDS message_runtime geometry_msgs 
    #INCLUDE_DIRS include
    #LIBRARIES # TODO
)

install (DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
