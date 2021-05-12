#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <memory>

#include <iostream>

#include "SerialPort.h"


SerialPort::SerialPort(const char *comPort, const speed_t baud)
{
  struct termios tty;
  char err[80];  

  _fd = open(comPort, O_RDWR | O_NOCTTY | O_SYNC);// | O_NONBLOCK);
  if(_fd == -1)
  {
    strcpy(err, "open_port: Unable to open ");
    strcat(err, comPort);

    perror(err);
    exit(1);
  }
  fprintf(stderr, "%s has been successfully opened\n", comPort);

  memset(&tty, 0, sizeof(tty));  //tty to 0
  if(tcgetattr(_fd, &tty) != 0)
  {
    perror("error from tcgetattr");
  }

  cfsetispeed(&tty, baud);  //set baud input
  cfsetospeed(&tty, baud);  //set baud output

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // ignore break signal
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  /*MIN  ==  0;  TIME  >  0: TIME specifies the limit for a timer in tenths of a second.  The
   timer is started when read(2) is called.  read(2) returns either when at least  one  byte
   of  data is available, or when the timer expires.  If the timer expires without any input
   becoming available, read(2) returns 0*/
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  //a thenth of a second read-timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB; // no stop bit
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag &= ~IGNCR;  // turn off ignore \r
  tty.c_iflag &= ~INLCR;  // turn off translate \n to \r
  tty.c_iflag &= ~ICRNL;  // turn off translate \r to \n

  tty.c_oflag &= ~ONLCR;  // turn off map \n  to \r\n
  tty.c_oflag &= ~OCRNL;  // turn off map \r to \n
  tty.c_oflag &= ~OPOST;  // turn off implementation defined output processing

  if(tcsetattr(_fd, TCSANOW, &tty) != 0)
  {
    perror("error calling tcsetattr");
    exit(1);
  }
}

SerialPort::~SerialPort()
{

  close(_fd);
  fprintf(stderr, "serial port closed \n");

}

bool SerialPort::send(float rpm[4])
{
  char tx_buffer[8];

  // split the to send data in 1 Byte pkgs -> send 8 bytes
  // convert to high and low byte

  short txval = rpm[0];
  tx_buffer[0] = txval & 0x00FF;
  tx_buffer[1] = (txval>>8) & 0x00FF;
  txval = rpm[1];
  tx_buffer[2] = txval & 0x00FF;
  tx_buffer[3] = (txval>>8) & 0x00FF;
  txval = rpm[2];
  tx_buffer[4] = txval & 0x00FF;
  tx_buffer[5] = (txval>>8) & 0x00FF;
  txval = rpm[3];
  tx_buffer[6] = txval & 0x00FF;
  tx_buffer[7] = (txval>>8) & 0x00FF;
  
  int sendBytes = write(_fd, (char*)tx_buffer, 8);
  
  if(sendBytes == 8)
    return true;
  else
    return false;
}

//bool SerialPort::receive(char* msg, unsigned int len)
//bool SerialPort::receive(std::shared_ptr<ReturnData> returnDataObj)
bool SerialPort::receive(ReturnData& returnDataObj)
{
  char rx_buffer[1024];
  unsigned int msgLen = 10;

  //serial_->receive(rx_buffer, 10)


  ssize_t bytesRead = 0;
  ssize_t bytesToRead = msgLen*sizeof(*rx_buffer);
  ssize_t bytesAvailable = 0;
  int cycles = 0;
  int maxCycles = 10;
  
  // get the Data
  while(bytesRead!=bytesToRead && cycles<maxCycles)
  {
    errno = 0;
    bytesAvailable = read(_fd, &(rx_buffer[bytesRead]), bytesToRead-bytesRead);
    bytesRead += bytesAvailable;
    usleep(100);
    cycles++;
    //std::cout << "available: " << bytesAvailable << " errno: " <<  errno << " read: " << bytesRead << " " << bytesToRead << std::endl;
  }


  // Fuse the 1byte Data to the used datatype
  short sval = (rx_buffer[0] | (((int)rx_buffer[1])<<8)) & 0xFF00;
  returnDataObj.rpmReturn[0] = ((float)sval) / 10.f;
  sval = (rx_buffer[2] | (((int)rx_buffer[3])<<8)) & 0xFF00;
  returnDataObj.rpmReturn[1] = ((float)sval) / 10.f;
  sval = (rx_buffer[4] | (((int)rx_buffer[5])<<8)) & 0xFF00;
  returnDataObj.rpmReturn[2] = ((float)sval) / 10.f;
  sval = (rx_buffer[6] | (((int)rx_buffer[7])<<8)) & 0xFF00;
  returnDataObj.rpmReturn[3] = ((float)sval) / 10.f;
  returnDataObj.voltage = ((float) ((rx_buffer[8] & 0x00FF) | (((int)rx_buffer[9])<<8) & 0xFF00)) / 100.f;
  
  if(cycles>=maxCycles)
  {
    std::cout << "timeout while receiving data" << std::endl;
    return false;
  }

  if(bytesRead == bytesToRead)
  {
    return true;
  }
  else
  {
    return false;
  }
}
