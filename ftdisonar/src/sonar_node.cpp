// ros lib
#include "ros/ros.h"
#include "sonar/serial.h"

// String libs
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <sstream>

// Serial libs
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitionss

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64>("sonar_distance", 1);
  ros::Publisher reading_pub = n.advertise<std_msgs::Int32>("sonar_reading", 1);
  ros::Rate loop_rate(20);
  std_msgs::Float64 distance;
  std_msgs::Int32 reading;

  int fd = openSerial("/dev/ttyUSB0");
  confSerial(fd);

  char read[4];
  int count = 0, good_readings = 0;
  float inches = 0;
  float meters = 0;

  std::stringstream stream;

  while (ros::ok())
  {
    count++;
    stream.str(std::string());

    // read serial
    readSerial(fd, stream);
    ROS_INFO("reading: %s", stream.str().c_str());
    stream >> read;

    // treat
    if (read[0] == 'R')
    {
      inches = atoi(&read[1]);
      meters = inches * 0.0254;
      good_readings++;
    }
    else
    {
      ROS_WARN("bad reading: %s", stream.str().c_str());
    }

    // publish
    distance.data = meters;
    pub.publish(distance);
    if (count == 20)
    {
      ROS_INFO("%d/%d good readings in last second", good_readings, count);
      reading.data = good_readings;
      reading_pub.publish(reading);
      good_readings = 0;
      count = 0;
    }

    // ROS stuff
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}