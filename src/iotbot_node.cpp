#include <ros/ros.h>
#include <ros/console.h>

#include "IotBot.h"

/**
 * @author Stefan May
 * @date 02.10.2020
 * @brief ROS node for the IOT2050 robot shield
 **/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "iotbot_node");
  ChassisParams chassisParams;
  MotorParams motorParams;

  // Assign motor channels to motor/wheel mounting
  ros::NodeHandle nh("~");

  nh.param("track",          chassisParams.track,               0.3f);
  nh.param("wheelBase",      chassisParams.wheelBase,           0.3f);
  nh.param("wheelDiameter",  chassisParams.wheelDiameter,       0.12f);
  nh.param("chFrontLeft",    chassisParams.chFrontLeft,         1);
  nh.param("chFrontRight",   chassisParams.chFrontRight,        0);
  nh.param("chRearLeft",     chassisParams.chRearLeft,          3);
  nh.param("chRearRight",    chassisParams.chRearRight,         2);
  nh.param("direction",      chassisParams.direction,           1);
  nh.param("gearRatio",      motorParams.gearRatio,             51.f);
  nh.param("encoderRatio",   motorParams.encoderRatio,          4096.f);
  nh.param("rpmMax",         motorParams.rpmMax,                80.f);

  IotBot robot(chassisParams, motorParams);
  robot.run();
}
