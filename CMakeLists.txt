cmake_minimum_required(VERSION 2.8.3)
project(labrom_sensors)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  image_transport
  usb_cam
)

find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

#############
## Testing ##
#############


