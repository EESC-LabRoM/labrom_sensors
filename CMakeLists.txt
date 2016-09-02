cmake_minimum_required(VERSION 2.8.3)
project(labrom_sensors)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
  image_transport
	sensor_msgs
  usb_cam
)

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_demux_node nodes/image_demux.cpp)
target_link_libraries(image_demux_node ${catkin_LIBRARIES})