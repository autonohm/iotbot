#ifndef __IOTSHIELD_H
#define __IOTSHIELD_H

#include <vector>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"

namespace iotbot
{

#define CMD_AUX1            0x40
#define CMD_AUX2            0x41
#define CMD_LIGHTS_OFF      0x42
#define CMD_DIM_LIGHT       0x43 // Dimmed headlight
#define CMD_HIGH_BEAM       0x44 // High beam headlight
#define CMD_FLASH_ALL       0x45 // Flash lights
#define CMD_FLASH_LEFT      0x46 // Flash lights
#define CMD_FLASH_RIGHT     0x47 // Flash lights
#define CMD_PULSATION       0x48 // Pulsation

enum eLighting {lightsOff    = CMD_LIGHTS_OFF,
					 dimLight     = CMD_DIM_LIGHT,
                beamLight    = CMD_HIGH_BEAM,
                warningLight = CMD_FLASH_ALL,
                flashLeft    = CMD_FLASH_LEFT,
                flashRight   = CMD_FLASH_RIGHT,
                pulsation    = CMD_PULSATION};

class IOTShield
{
public:
   IOTShield();
   
   ~IOTShield();

   bool setRPM(float rpm[4]);

   bool setLighting(eLighting light);
   
   bool setAUX1(bool on);
   
   bool setAUX2(bool on);
   
   float getSystemVoltage();
   
   const std::vector<float> getRangeMeasurements();
   
private:

   void sendReceive();

   mraa::Uart* _uart;

   char _txBuf[11];
   
   char _rxBuf[32];
   
   float _systemVoltage;
   
   std::vector<float> _ranges;
};

} // namespace

#endif //__IOTSHIELD_H
