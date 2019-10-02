cmake_minimum_required(VERSION 2.8.3)
project(spacenav_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  trajectory_msgs
  spacenav_node
  urdf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp sensor_msgs trajectory_msgs spacenav_node
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_controller
  src/controller.cpp
)

add_executable(${PROJECT_NAME}_node src/spacenav_control_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_controller ${catkin_LIBRARIES})
