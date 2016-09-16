// ros lib
#include "ros/ros.h"
#include "sonar/serial.h"

// String libs
#include "std_msgs/Float64.h"
#include <sstream>

// Serial libs
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitionss

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("sonar_distance", 1);
  ros::Rate loop_rate(20);
  std_msgs::Float64 distance;

  char read[4];
  int fd = openSerial("/dev/ttyUSB0");
  float inches = 0;
  float meters = 0;
  confSerial(fd);

  std::stringstream stream;

  while (ros::ok())
  {
    stream.str(std::string());

    // read serial
    readSerial(fd, stream);
    stream >> read;

    // treat
    if (read[0] == 'R')
    {
      inches = atoi(&read[1]);
      meters = inches * 0.0254;
    }

    // publish
    distance.data = meters;
    chatter_pub.publish(distance);

    // ROS stuff
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}