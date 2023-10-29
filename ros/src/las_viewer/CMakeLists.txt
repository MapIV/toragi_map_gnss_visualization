cmake_minimum_required(VERSION 2.8.3)
project(las_viewer)

find_package(libLAS REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  tf
  sensor_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(las_viewer_node src/las_viewer_node.cpp)
target_link_libraries(las_viewer_node ${catkin_LIBRARIES} ${libLAS_LIBRARIES})
add_dependencies(las_viewer_node ${catkin_EXPORTED_TARGETS})
