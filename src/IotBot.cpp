#include <iostream>

#include "IotBot.h"

using namespace std;

namespace iotbot
{

IotBot::IotBot(ChassisParams &chassisParams, MotorParams &motorParams)
{
  _motorParams  = new MotorParams(motorParams);
  _chassisParams = chassisParams;
  // ensure that the direction parameter is set properly (either 1 or -1)
  if(_chassisParams.direction>0) _chassisParams.direction = 1;
  else _chassisParams.direction = -1;

  _shield = new IOTShield();
  _shield->enable(); 
  _shield->setGearRatio(_motorParams->gearRatio);
  _shield->setTicksPerRev(_motorParams->encoderRatio);
  _shield->setKp(0);
  _shield->setKi(20);
  _shield->setKd(0.0);
  _shield->setLighting(iotbot::dimLight);

  _rad2rpm          = (chassisParams.wheelBase+chassisParams.track)/chassisParams.wheelDiameter; // (lx+ly)/2 * 1/r
  _rpm2rad          = 1.0 / _rad2rpm;
  _ms2rpm           = 60.0/(chassisParams.wheelDiameter*M_PI);
  _rpm2ms           = 1.0 / _ms2rpm;
  _vMax             = motorParams.rpmMax * _rpm2ms;
  _omegaMax         = motorParams.rpmMax * _rpm2rad;

  _joySub    = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &IotBot::joyCallback, this);
  _velSub    = _nh.subscribe<geometry_msgs::Twist>("vel/teleop", 1, &IotBot::velocityCallback, this);
 
  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;

  cout << "Initialized IotBot with vMax: " << _vMax << " m/s" << endl;
}

IotBot::~IotBot()
{
  _shield->setLighting(iotbot::pulsation);
  delete _shield;
  delete _motorParams;
}

void IotBot::run()
{
  ros::Rate rate(100);
  _lastCmd = ros::Time::now();

  bool run = true;

  float r[4];
  float voltage;

  int state = 0;


  unsigned int cnt = 0;
  while(run)
  {
    ros::spinOnce();

    ros::Duration dt = ros::Time::now() - _lastCmd;
    bool lag = (dt.toSec()>0.5);
    if(lag)
    {
      ROS_WARN_STREAM("Lag detected ... deactivate motor control");
      _shield->disable();
    }
    else
    {
      _shield->setRPM(_rpm);
    }
    
    rate.sleep();

    run = ros::ok();

  }
}

void IotBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Assignment of joystick axes to motor commands
  float fwd      = joy->axes[1];            // Range of values [-1:1]
  float left     = joy->axes[0];            // Range of values [-1:1]
  float turn     = joy->axes[2];            // Range of values [-1:1]
  float throttle = (joy->axes[3]+1.0)/2.0;  // Range of values [0:1]

  static int32_t btn0Prev   = joy->buttons[0];
  static int32_t btn4Prev   = joy->buttons[4];
  static int32_t btn5Prev   = joy->buttons[5];
  
  if(joy->buttons[0] && !btn0Prev)
     _shield->setLighting(iotbot::beamLight);
  if(joy->buttons[4] && !btn4Prev)
     _shield->setLighting(iotbot::flashLeft);
  if(joy->buttons[5] && !btn5Prev)
     _shield->setLighting(iotbot::flashRight);
     
  btn0Prev   = joy->buttons[0];
  btn4Prev   = joy->buttons[4];
  btn5Prev   = joy->buttons[5];
  
  if(!btn0Prev && !btn4Prev && !btn5Prev)
       _shield->setLighting(iotbot::dimLight);
  
  float vFwd  = throttle * fwd  * _vMax;
  float vLeft = throttle * left * _vMax;
  float omega = throttle * turn * _omegaMax;

  controlMotors(vFwd, vLeft, omega);
}

void IotBot::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

void IotBot::controlMotors(float vFwd, float vLeft, float omega)
{
  float rpmFwd   = vFwd  * _ms2rpm;
  float rpmLeft  = vLeft * _ms2rpm;
  float rpmOmega = omega * _rad2rpm;
  rpmLeft = 0; // deactivated movement in y-direction
  //cout << "vFwd: " << vFwd << "m/s, vLeft: " << vLeft << "m/s, omega: " << omega << endl;
  //cout << "rpmFwd: " << rpmFwd << ", rpmLeft: " << rpmLeft << ", rpmOmega: " << rpmOmega << endl;

  // leading signs -> see derivation: Stefan May, Skriptum Mobile Robotik
  _rpm[_chassisParams.chFrontLeft]  =  rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.chFrontRight] = -rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.chRearLeft]   =  rpmFwd + rpmLeft - rpmOmega;
  _rpm[_chassisParams.chRearRight]  = -rpmFwd + rpmLeft - rpmOmega;

  // possibility to flip directions
  _rpm[0] *= _chassisParams.direction;
  _rpm[1] *= _chassisParams.direction;
  _rpm[2] *= _chassisParams.direction;
  _rpm[3] *= _chassisParams.direction;

  // Normalize values, if any value exceeds the maximum
  float rpmMax = std::abs(_rpm[0]);
  for(int i=1; i<4; i++)
  {
    if(std::abs(_rpm[i]) > _motorParams->rpmMax)
      rpmMax = std::abs(_rpm[i]);
  }

  if(rpmMax > _motorParams->rpmMax)
  {
    float factor = _motorParams->rpmMax / rpmMax;
    for(int i=0; i<4; i++)
      _rpm[i] *= factor;
  }

  _lastCmd = ros::Time::now();
}

} // end namespace
