// Serial libs
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitionss

int openSerial(char *portName)
{
  int fd = open(portName, O_CREAT | O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK, S_IRUSR | S_IWUSR);
  if (!fd)
  {
    return -1;
  }
  else
  {
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
  int end_count = 0, result;
  unsigned char data;
  while (end_count < 2)
  {
    result = read(fd, &data, 1);
    if (result > 0)
    {
      if (data == ']')
      {
        end_count++;
      }
      ss << data;
    }
    else
    {
      break;
    }
  }
}
