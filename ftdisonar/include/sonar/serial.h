// Serial libs
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <iostream>

int openSerial(char *portName)
{
  int fd = open(portName, O_CREAT | O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK, S_IRUSR | S_IWUSR);
  if (!fd)
  {
    return -1;
    ROS_ERROR("Error trying to open serial port, verify permission");
  }
  else
  {
    ROS_INFO("Serial port ok!");
  }
  return fd;
}

void confSerial(int fd)
{
  struct termios port_settings;
  cfsetispeed(&port_settings, B9600);
  port_settings.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
  port_settings.c_iflag = IGNPAR | IXOFF;
  port_settings.c_oflag = 0;
  port_settings.c_lflag = 0;
  tcsetattr(fd, TCSANOW, &port_settings);
}

void readSerial(int fd, std::stringstream &ss)
{
  int result;
  unsigned char data[10];
  result = read(fd, &data, 10);
  ss << data;
}
