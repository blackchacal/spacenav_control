cmake_minimum_required(VERSION 2.8.3)
project(spacenav_control)

# add_compile_options(-std=c++11)
add_compile_options(-std=c++14 -Wall -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  spacenav_node
  urdf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_controller
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs trajectory_msgs spacenav_node
)

###########
## Build ##
###########
  
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/.cmake)
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${EIGEN_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_controller src/controller.cpp)

add_executable(${PROJECT_NAME}_node src/spacenav_control_node.cpp)

target_link_libraries(${PROJECT_NAME}_controller ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_controller ${Boost_LIBRARIES} ${catkin_LIBRARIES})
