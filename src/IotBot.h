#ifndef IOTBOT_H_
#define IOTBOT_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include "IOTShield.h"

using namespace std;

namespace iotbot
{

/**
 * @struct Structure for encapsulating motor parameters
 * @author Stefan May
 * @date 08.05.2021
 */
struct MotorParams
{
  float            gearRatio;
  float            encoderRatio;
  float            rpmMax;

  /**
   * Standard constructor assigns default parameters
   */
  MotorParams()
  {
    gearRatio      = 0.f;
    encoderRatio   = 0.f;
    rpmMax         = 0.f;
  }

  /**
   * Copy constructor
   * @param[in] p parameter instance to be copied
   */
  MotorParams(const MotorParams &p)
  {
    gearRatio      = p.gearRatio;
    encoderRatio   = p.encoderRatio;
    rpmMax         = p.rpmMax;
  }
};

/**
 * @struct Structure for encapsulating parameters of robot chassis
 * @author Stefan May
 * @date 08.05.2021
 */
struct ChassisParams
{
  float track;
  float wheelBase;
  float wheelDiameter;
  int   chFrontLeft;
  int   chFrontRight;
  int   chRearLeft;
  int   chRearRight;
  int   direction;

  ChassisParams()
  {
    track               = 0.f;
    wheelBase           = 0.f;
    wheelDiameter       = 0.f;
    chFrontLeft         = 0;
    chFrontRight       = 0;
    chRearLeft          = 0;
    chRearRight         = 0;
    direction           = 0;
  }
};

/**
 * @class Main class for IotBot
 * @author Stefan May
 * @date 08.05.2021
 */
class IotBot
{
public:

  /**
   * Standard Constructor
   * @params[in] chassisParams chassis parameters, including the map for assigning channels to position of wheels
   * @params[in] motorParams motor parameters
   */
  IotBot(ChassisParams &chassisParams, MotorParams &motorParams);

  /**
   * Destructor
   */
  ~IotBot();

  /**
   * ROS main loop (blocking method)
   */
  void run();

private:

  /**
   * ROS joystick callback
   * @param[in] joy message with joystick command
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * ROS command velocity callback
   * @param[in] cmd message with velocity command
   */
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd);

  /**
   * Normalize motion command and assign it to the channels
   * @param[in] vFwd forward velocity (x-axis)
   * @param[in] vLeft velocity to the left (y-axis)
   * @param[in] omega angular velocity (around z-axis)
   */
  void controlMotors(float vFwd, float vLeft, float omega);

  ros::NodeHandle        _nh;
  ros::Subscriber        _joySub;
  ros::Subscriber        _velSub;

  ChassisParams          _chassisParams;
  MotorParams*           _motorParams;
  IOTShield*             _shield;

  // revolutions per minute for each channel
  float                  _rpm[4];

  // maximum velocity [m/s]
  float                  _vMax;

  // maximum rotating rate [rad]
  float                  _omegaMax;

  // conversion from [m/s] to revolutions per minute [RPM]
  float                  _ms2rpm;

  // conversion from revolutions per minute [RPM] to [m/s]
  float                  _rpm2ms;

  // conversion from [rad/s] to revolutions per minute [RPM]
  float                  _rad2rpm;

  // conversion from revolutions per minute [RPM] to [rad/s]
  float                  _rpm2rad;

  // time elapsed since last call
  ros::Time              _lastCmd;
};

} // end namespace

#endif
