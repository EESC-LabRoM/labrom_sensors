// ROS libraries
#include "ros/ros.h"
// ROS message libraries
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
// Serial data reading
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/signal.h>
#include <sys/types.h>
// Serial port configuration
#define BAUDRATE B9600
#define FALSE 0
#define TRUE 1
//
volatile int STOP = FALSE;
void signal_handler_IO(int status);
int wait_flag = TRUE;
int status;
//
int main(int argc, char *argv[])
{
  // Initialize ROS within 
  ros::init(argc, argv, "sonar");
  // Node Handle
  ros::NodeHandle nh, pnh("~");
  // Publishers
  ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>("distance", 1);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose",1);

  // ROS messages
  std_msgs::Float64 distance;
  geometry_msgs::PoseWithCovarianceStamped pose;

  // Laoding parameters
  std::string port;
  pnh.param<std::string>("port",port,"USB0");

  // Initializing pose message
  pose.header.frame_id = "floor_sonar";
  pose.header.seq = 0;
  pose.pose.covariance[6*2+2] = 0.01; 
  
  char ch;
  char readSonar[5];
  int inches, index = 0;

  int fd, res, i;
  struct termios newtio;
  struct sigaction saio;
  char buf[255];
  //
  //open the device in non-blocking way (read will return immediately)
  port = "/dev/tty"+port;
  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    perror(port.c_str());
    exit(1);
  }
  //
  //install the serial handler before making the device asynchronous
  saio.sa_handler = signal_handler_IO;
  sigemptyset(&saio.sa_mask); //saio.sa_mask = 0;
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO, &saio, NULL);
  //
  // allow the process to receive SIGIO
  fcntl(fd, F_SETOWN, getpid());
  //
  // make the file descriptor asynchronous
  fcntl(fd, F_SETFL, FASYNC);
  //
  // set new port settings for canonical input processing
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VMIN] = 1;
  newtio.c_cc[VTIME] = 0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  //
  // loop while waiting for input. normally we would do something useful here
  while (ros::ok())
  {
    //
    // read characters typed by user in non-blocking way
    ch = getchar_unlocked();
    if (ch > 0)
      write(fd, &ch, 1);
    //
    // after receiving SIGIO, wait_flag = FALSE, input is available and can be read */
    if (wait_flag == FALSE) //if input is available
    {
      res = read(fd, buf, 255);
      if (res > 0)
      {
        for (i = 0; i < res; i++) //for all chars in string
        {
          if (buf[i] == 'R')
          {
            // Reading sonar
            inches = atoi(&readSonar[1]);
            // Float message
            distance.data = inches * 0.0254;
            // Pose message
            pose.pose.pose.position.z = distance.data;
            pose.header.seq += 1;
            pose.header.stamp = ros::Time::now();
            
            // Publishing message
            dist_pub.publish(distance);
            pose_pub.publish(pose);

            index = 0;
          }
          readSonar[index] = buf[i];
          index++;
        }
      }
      wait_flag = TRUE; /* wait for new input */
    }
    ros::spinOnce();
  }
  close(fd);
}
//
/***************************************************************************
* signal handler. sets wait_flag to FALSE, to indicate above loop that     *
* characters have been received.                                           *
***************************************************************************/
//
void signal_handler_IO(int status)
{
  //printf("received SIGIO signal.\n");
  wait_flag = FALSE;
}