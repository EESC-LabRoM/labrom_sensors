cmake_minimum_required(VERSION 2.8.3)
project(labrom_camera)

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
  ${catkin_INCLUDE_DIRS}
)

