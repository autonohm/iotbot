#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"     
#include "IotBot.h"

/**
 * @author Stefan May, David Grenner
 * @date 19.10.2020
 * @brief ROS2 node for the IOT2050 robot shield
 **/

class Iotbot_node : public rclcpp::Node 
{
  ChassisParams chassisParams;
  MotorParams motorParams;

  public:
  Iotbot_node() : Node ("iotbot_node")
  {
    // Assign motor channels to motor/wheel mounting
    // ros::NodeHandle nh("~");

    // //nh.param = nh.getParam
    // nh.param("track",          chassisParams.track,               0.3f);
    // nh.param("wheelBase",      chassisParams.wheelBase,           0.3f);
    // nh.param("wheelDiameter",  chassisParams.wheelDiameter,       0.12f);
    // nh.param("chFrontLeft",    chassisParams.chFrontLeft,         1);
    // nh.param("chFrontRight",   chassisParams.chFrontRight,        0);
    // nh.param("chRearLeft",     chassisParams.chRearLeft,          3);
    // nh.param("chRearRight",    chassisParams.chRearRight,         2);
    // nh.param("direction",      chassisParams.direction,           1);
    // nh.param("gearRatio",      motorParams.gearRatio,             51.f);
    // nh.param("encoderRatio",   motorParams.encoderRatio,          4096.f);
    // nh.param("rpmMax",         motorParams.rpmMax,                80.f);

    chassisParams.track =               0.3f;
    chassisParams.wheelBase =           0.3f;
    chassisParams.wheelDiameter =       0.12f;
    chassisParams.chFrontLeft =         1;
    chassisParams.chFrontRight =        0;
    chassisParams.chRearLeft =          3;
    chassisParams.chRearRight =         2;
    chassisParams.direction =           1;
    motorParams.gearRatio =             51.f;
    motorParams.encoderRatio =          4096.f;
    motorParams.rpmMax =                80.f;


    IotBot robot(chassisParams, motorParams);
    robot.run();
  }

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Iotbot_node>()); 
  return 0; 
}
