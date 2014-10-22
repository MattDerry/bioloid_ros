#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "zgb_hal.h"

int gSocket_fd  = -1;
long  glStartTime = 0;
float gfRcvWaitTime = 0.0f;
float gfByteTransTime = 0.0f;

char  gDeviceName[20];

int zgb_hal_open( int devIndex, float baudrate )
{
  // Opening device
  // devIndex: Device index
  // baudrate: Real baudrate (ex> 115200, 57600, 38400...)
  // Return: 0(Failed), 1(Succeed)

  struct termios newtio;
  char dev_name[100] = {0, };

  sprintf(dev_name, "/dev/rfcomm%d", devIndex);

  fprintf(stderr, "connecting to %s\n", dev_name);

  strcpy(gDeviceName, dev_name);
  memset(&newtio, 0, sizeof(newtio));
  zgb_hal_close();
  
  if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
    fprintf(stderr, "first device openerror\n");
    fprintf(stderr, "device open error: %s\n", dev_name);
    goto ZGB_HAL_OPEN_ERROR;
  }

  fprintf(stderr, "gSocket_fd %d\n", gSocket_fd);

  newtio.c_cflag    = B57600|CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;  // time-out 값 (TIME * 0.1초) 0 : disable
  newtio.c_cc[VMIN] = 0;  // MIN 은 read 가 return 되기 위한 최소 문자 개수

  int res = tcflush(gSocket_fd, TCIFLUSH);

  fprintf(stderr, "flush returned %d\n", res);

  res = tcsetattr(gSocket_fd, TCSANOW, &newtio);

  fprintf(stderr, "set attributes returned %d\n", res);

  
  if(gSocket_fd == -1)
    return 0;
  
  fprintf(stderr, "gSocket_fd %d\n", gSocket_fd);
  
  zgb_hal_close();
  
  gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
  
  strcpy(gDeviceName, dev_name);
  memset(&newtio, 0, sizeof(newtio));
  zgb_hal_close();
  
  if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
    fprintf(stderr, "device open error: %s\n", dev_name);
    goto ZGB_HAL_OPEN_ERROR;
  }

  newtio.c_cflag    = B57600|CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;  // time-out 값 (TIME * 0.1초) 0 : disable
  newtio.c_cc[VMIN] = 0;  // MIN 은 read 가 return 되기 위한 최소 문자 개수

  tcflush(gSocket_fd, TCIFLUSH);
  tcsetattr(gSocket_fd, TCSANOW, &newtio);
  
  return 1;

ZGB_HAL_OPEN_ERROR:
  zgb_hal_close();
  return 0;
}

void zgb_hal_close()
{
  if(gSocket_fd != -1)
    close(gSocket_fd);
  gSocket_fd = -1;
}

int zgb_hal_tx( unsigned char *pPacket, int numPacket )
{
  return write(gSocket_fd, pPacket, numPacket);
}

int zgb_hal_rx( unsigned char *pPacket, int numPacket )
{
  memset(pPacket, 0, numPacket);
  return read(gSocket_fd, pPacket, numPacket);
}

int zgb_hal_flush()
{
  return tcflush(gSocket_fd, TCIFLUSH);
}
