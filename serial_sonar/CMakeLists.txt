cmake_minimum_required(VERSION 2.8.3)
project(serial_sonar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(sonar_node src/sonar_node.cpp)
target_link_libraries(sonar_node ${catkin_LIBRARIES})