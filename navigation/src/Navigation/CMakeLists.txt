cmake_minimum_required(VERSION 2.8)
project(navigation)


set(PACKAGE_DEPENDENCIES
  octomap_msgs
  roscpp
  rospy
  pcl_ros
  pcl_conversions
  std_msgs
  message_generation
  tf
  visualization_msgs
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)


generate_messages( DEPENDENCIES
  std_msgs
  geometry_msgs )

  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs )

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  /usr/local/lib/libmosquitto.so.1 
  /usr/lib/x86_64-linux-gnu/libjsoncpp.so
)

add_library(webserver src/webserver.cpp )

## Declare a C++ library
add_library(global_planner
  src/node.cpp
  src/cell.cpp
  src/global_planner.cpp
)

add_executable(main src/main.cpp src/global_planner_node.cpp src/global_planner_fdu.cpp)
target_link_libraries(main webserver global_planner ${LINK_LIBS})

add_executable(mock_data_node src/mock_data_node.cpp)
target_link_libraries(mock_data_node webserver ${LINK_LIBS})


