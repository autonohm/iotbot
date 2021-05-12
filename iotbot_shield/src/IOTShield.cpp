#include "IOTShield.h"
#include <iostream>

namespace iotbot
{

const char* devPath = "/dev/ttyS1";

IOTShield::IOTShield()
{
   _ranges.resize(4);
   
   _uart = new mraa::Uart(devPath);

   if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
      std::cerr << "Error setting flow control UART" << std::endl;
   }
   
   _uart->flush();
}
   
IOTShield::~IOTShield()
{
   delete _uart;
}

bool IOTShield::setRPM(float rpm[4])
{
   return 0;
}

bool IOTShield::setLighting(eLighting light)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = light;
	for(int i=2; i<10; i++)
	   _txBuf[i] = i;
	_txBuf[10] = 0xEE;
   sendReceive();
   return 0;
}

bool IOTShield::setAUX1(bool on)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_AUX1;
   _txBuf[2] = (on ? 0x01 : 0x00);
	for(int i=3; i<10; i++)
	   _txBuf[i] = 0;
	_txBuf[10] = 0xEE;
   sendReceive();
   return 0;
}
   
bool IOTShield::setAUX2(bool on)
{
   _txBuf[0] = 0xFF;
   _txBuf[1] = CMD_AUX2;
   _txBuf[2] = on;
	for(int i=3; i<10; i++)
	   _txBuf[i] = 0;
	_txBuf[10] = 0xEE;
   sendReceive();
   return 0;
}

float IOTShield::getSystemVoltage()
{
   return _systemVoltage;
}
   
const std::vector<float> IOTShield::getRangeMeasurements()
{
   return _ranges;
}

void IOTShield::sendReceive()
{
   _uart->write((char*)_txBuf, 11);
   _uart->read(_rxBuf, 32);
   unsigned int voltage = _rxBuf[30];
   voltage = voltage << 8;
   voltage |= _rxBuf[29];
   _systemVoltage = (float) voltage / 100.f;
   
   for(int i=0; i<4; i++)
   {
      unsigned short distanceInMM = _rxBuf[22+2*i];
      distanceInMM = distanceInMM << 8;
      distanceInMM |= _rxBuf[21+2*i];
      _ranges[i] = (float)distanceInMM;
      _ranges[i] /= 1000.f;
   }
   
}

} // namespace
