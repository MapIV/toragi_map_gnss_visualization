cmake_minimum_required(VERSION 2.8.3)
project(gnss_viewer)

# set(CMAKE_CXX_FLAGS "-O2 -std=c++17 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  nmea_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nmea_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

message(${PROJECT_SOURCE_DIR})

add_executable(gnss_viewer_node src/gnss_viewer_node.cpp)
target_link_libraries(gnss_viewer_node ${catkin_LIBRARIES} ${libLAS_LIBRARIES})
add_dependencies(gnss_viewer_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  gnss_viewer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)