#ifndef __IOTSHIELD_H
#define __IOTSHIELD_H

#include <vector>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"

namespace iotbot
{

// Common parameters
#define CMD_ENABLE          0x01
#define CMD_DISABLE         0x02
#define CMD_SETTIMEOUT      0x03
#define CMD_SETPWMMAX       0x04
#define CMD_SENDRPM         0x05
#define CMD_SENDPOS         0x06
#define CMD_INVERTENC       0x07

// Operating commands
#define CMD_SETPWM          0x10
#define CMD_SETRPM          0x11
#define CMD_FREQ            0x12
#define CMD_SYNC            0x13

// Closed/Open loop controller parameters
#define CMD_CTL_KP          0x20
#define CMD_CTL_KI          0x21
#define CMD_CTL_KD          0x22
#define CMD_CTL_ANTIWINDUP  0x23
#define CMD_CTL_INPUTFILTER 0x24
#define CMD_CTL_ENCLOWPASS  0x25

// Platform parameters
#define CMD_GEARRATIO       0x30
#define CMD_TICKSPERREV     0x31

#define CMD_AUX1            0x40
#define CMD_AUX2            0x41
#define CMD_LIGHTS_OFF      0x42
#define CMD_DIM_LIGHT       0x43 // Dimmed headlight
#define CMD_HIGH_BEAM       0x44 // High beam headlight
#define CMD_FLASH_ALL       0x45 // Flash lights
#define CMD_FLASH_LEFT      0x46 // Flash lights
#define CMD_FLASH_RIGHT     0x47 // Flash lights
#define CMD_PULSATION       0x48 // Pulsation
#define CMD_ROTATION        0x49 // Rotation light, e.g. police light in blue
#define CMD_RUNNING         0x4A // Running light

enum eLighting {lightsOff    = CMD_LIGHTS_OFF,
                dimLight     = CMD_DIM_LIGHT,
                beamLight    = CMD_HIGH_BEAM,
                warningLight = CMD_FLASH_ALL,
                flashLeft    = CMD_FLASH_LEFT,
                flashRight   = CMD_FLASH_RIGHT,
                pulsation    = CMD_PULSATION,
                rotation     = CMD_ROTATION,
                running      = CMD_RUNNING};

class IOTShield
{
public:
   IOTShield();
   
   ~IOTShield();

   bool enable();
   
   bool disable();

   bool setGearRatio(float gearRatio);
   
   bool setTicksPerRev(float ticksPerRev);

   bool setKp(float kp);
   
   bool setKi(float ki);
      
   bool setKd(float kd);

   bool setControlFrequency(uint32_t freq);

   bool setLowPassSetPoint(float weight);

   bool setLowPassEncoder(float weight);

   bool setPWM(int8_t pwm[4]);

   bool setRPM(float rpm[4]);

   const std::vector<float> getRPM();

   bool setLighting(eLighting light, unsigned char rgb[3]);
   
   bool setAUX1(bool on);
   
   bool setAUX2(bool on);
   
   float getSystemVoltage();
   
   const std::vector<float> getRangeMeasurements();

   const std::vector<float> getAcceleration();

   const std::vector<float> getAngularRate();
   
private:

   void sendReceive();

   mraa::Uart* _uart;

   char _txBuf[11];
   
   char _rxBuf[32];
   
   float _systemVoltage;
   
   std::vector<float> _rpm;
      
   std::vector<float> _ranges;
   
   std::vector<float> _acceleration;

   std::vector<float> _angularRate;

   double _timeCom;
   
};

} // namespace

#endif //__IOTSHIELD_H
