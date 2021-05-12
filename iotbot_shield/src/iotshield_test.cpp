#include "IOTShield.h"
#include <unistd.h>
#include <iostream>

/**
 * @author Stefan May
 * @date 26.04.2021
 * @brief 
 **/
int main(int argc, char* argv[])
{
   iotbot::IOTShield iot;
  
   std::cout << "AUX1 on" << std::endl;
   iot.setAUX1(true);
   usleep(500000);
   
   std::cout << "AUX2 on" << std::endl;
   iot.setAUX2(true);
   usleep(500000);

   std::cout << "AUX1 off" << std::endl;
   iot.setAUX1(false);
   usleep(500000);
   
   std::cout << "AUX2 off" << std::endl;
   iot.setAUX2(false);
   usleep(500000);

   for(int i=0; i<120; i++)
   {
      iot.setAUX2(false);
      std::vector<float> ranges = iot.getRangeMeasurements();
      std::cout << "Ranges: ";
      for(int j=0; j<4; j++)
         std::cout << ranges[j] << "m  ";
      std::cout << std::endl;
      usleep(25000);
   }

   std::cout << "Setting lights off" << std::endl;
   iot.setLighting(iotbot::lightsOff);
   usleep(1000000);
   
   std::cout << "Setting dimmed lights" << std::endl;
   iot.setLighting(iotbot::dimLight);
   usleep(1000000);
   
   std::cout << "Setting high beam lights" << std::endl;
   iot.setLighting(iotbot::beamLight);
   usleep(1000000);
   
   std::cout << "Setting warning lights" << std::endl;
   iot.setLighting(iotbot::warningLight);
   usleep(3000000);
   
   std::cout << "Setting flash left" << std::endl;
   iot.setLighting(iotbot::flashLeft);
   usleep(3000000);
   
   std::cout << "Setting flash right" << std::endl;
   iot.setLighting(iotbot::flashRight);
   usleep(3000000);
   
   std::cout << "Setting pulsation" << std::endl;
   iot.setLighting(iotbot::pulsation);
   usleep(3000000);
   
   std::cout << "Voltage: " << iot.getSystemVoltage() << "V" << std::endl;
  
   
   return 0;
}
