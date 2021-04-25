cmake_minimum_required(VERSION 2.8.3)
project(test_mavros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  control_toolbox
  eigen_conversions
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
  gazebo_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

include(EnableCXX11)

# Fix issue #546: control_toolbox changed API in Kinetic
if(${control_toolbox_VERSION} VERSION_LESS 1.14.0)
  message(STATUS "Use old control_toolbox API")
  add_definitions(-DCONTROL_TOOLBOX_PRE_1_14)
else()
  message(STATUS "Use new control_toolbox API")
endif()
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
  #INCLUDE_DIRS include
  #LIBRARIES sitl_test
  CATKIN_DEPENDS  control_toolbox eigen_conversions geometry_msgs mavros mavros_extras roscpp std_msgs tf2_ros
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  sitl_test_node
  src/sitl_test_node.cpp
  src/pid_controller.cpp
  src/offboard_control.cpp
)
add_dependencies(
  sitl_test_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(
  sitl_test_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
