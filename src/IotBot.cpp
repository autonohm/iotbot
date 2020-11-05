#include <iostream>
#include <iomanip> 

#include "IotBot.h"

using namespace std;

IotBot::IotBot(ChassisParams &chassisParams, MotorParams &motorParams) : Node("IotBot")
{
  _serial = new SerialPort("/dev/ttyS1", B115200);

  _motorParams  = new MotorParams(motorParams);
  _chassisParams = chassisParams;
  // ensure that the direction parameter is set properly (either 1 or -1)
  if(_chassisParams.direction>0) _chassisParams.direction = 1;
  else _chassisParams.direction = -1;

  _rad2rpm          = (chassisParams.wheelBase+chassisParams.track)/chassisParams.wheelDiameter; // (lx+ly)/2 * 1/r
  _rpm2rad          = 1.0 / _rad2rpm;
  _ms2rpm           = 60.0/(chassisParams.wheelDiameter*M_PI);
  _rpm2ms           = 1.0 / _ms2rpm;
  _vMax             = motorParams.rpmMax * _rpm2ms;
  _omegaMax         = motorParams.rpmMax * _rpm2rad;

  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;

  cout << "Initialized IotBot with vMax: " << _vMax << " m/s" << endl;
}

IotBot::~IotBot()
{
  delete _serial;
  delete _motorParams;
  cout << " delete _serial , delete _motorParams" << endl;
}

void IotBot::run()
{
  rclcpp::Rate rate(100);
  using builtin_interfaces::msg::Time;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  _lastCmd = ros_clock.now();

  bool run = true;

  float r[4];
  float voltage;

  char tx_buffer[8];
  char rx_buffer[1024];
  int state = 0;

  unsigned int cnt = 0;
  while(run)
  {
    auto Node = rclcpp::Node::make_shared("iot_sub_node");
    // initialize subscribers
    _joySub =  Node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&IotBot::joyCallback, this , std::placeholders::_1));
    _velSub = Node->create_subscription<geometry_msgs::msg::Twist>("vel/teleop", 1, std::bind(&IotBot::velocityCallback, this, std::placeholders::_1));

    //initialize Publisher
    _velPub = this->create_publisher<geometry_msgs::msg::Twist>("/vel/robot", 50); 

    rclcpp::spin_some(Node);
    rclcpp::Duration dt = ros_clock.now() - _lastCmd;
                  
    bool lag = (dt.nanoseconds()/1000000000>0.5);
      
    if(lag)
    {
      
      //ROS_WARN_STREAM("Lag detected ... deactivate motor control");
      tx_buffer[0] = 0;
      tx_buffer[1] = 0;
      tx_buffer[2] = 0;
      tx_buffer[3] = 0;
      tx_buffer[4] = 0;
      tx_buffer[5] = 0;
      tx_buffer[6] = 0;
      tx_buffer[7] = 0;
    }
    else
    {
      short txval = _rpm[0];
      tx_buffer[0] = txval & 0x00FF;
      tx_buffer[1] = (txval>>8) & 0x00FF;
      txval = _rpm[1];
      tx_buffer[2] = txval & 0x00FF;
      tx_buffer[3] = (txval>>8) & 0x00FF;
      txval = _rpm[2];
      tx_buffer[4] = txval & 0x00FF;
      tx_buffer[5] = (txval>>8) & 0x00FF;
      txval = _rpm[3];
      tx_buffer[6] = txval & 0x00FF;
      tx_buffer[7] = (txval>>8) & 0x00FF;
    }

    switch(state)
    {
      case 0:
        if(_serial->send((char*)tx_buffer, 8)==8)
        {
          if(cnt%10==0) std::cout << "# Sent 8 bytes of uart data" << std::endl;
          cnt++;
          state = 1;
        }
        break;
      case 1:
        if(_serial->receive(rx_buffer, 10))
        {
          short sval = (rx_buffer[0] | (((int)rx_buffer[1])<<8)) & 0xFF00;
          r[0] = ((float)sval) / 10.f;
          sval = (rx_buffer[2] | (((int)rx_buffer[3])<<8)) & 0xFF00;
          r[1] = ((float)sval) / 10.f;
          sval = (rx_buffer[4] | (((int)rx_buffer[5])<<8)) & 0xFF00;
          r[2] = ((float)sval) / 10.f;
          sval = (rx_buffer[6] | (((int)rx_buffer[7])<<8)) & 0xFF00;
          r[3] = ((float)sval) / 10.f;
          voltage = ((float) ((rx_buffer[8] & 0x00FF) | (((int)rx_buffer[9])<<8) & 0xFF00)) / 100.f;  //TODO check 
          
          if(cnt%10==0) std::cout << "# Received: ";
          if(cnt%10==0) std::cout << std::setprecision(2) << std::fixed << "r[0]=" << r[0] << "rpm, r[1]=" << r[1] << "rpm, r[2]=" << r[2]  << "rpm, r[3]=" << r[3] << "rpm";
          if(cnt%10==0) std::cout << std::setprecision(2) << std::fixed << ", Vin=" << voltage << "V" << std::endl;
          //TODO: publish r[0] bis r[3]

          //rpm to m/s
          float vFwd = 1/2 * (r[0] - r[1]) * _rpm2ms ;
          float omega = -1/2 * (r[0] + r[1]) * _rpm2rad;

          auto msgTwist = geometry_msgs::msg::Twist();

          msgTwist.linear.x = vFwd;
          msgTwist.linear.y = 0;
          msgTwist.linear.z = 0;
          msgTwist.angular.z = omega;
          _velPub->publish(msgTwist); 
        }
	state = 0;
	break;
       default:
         break;
    }
   run = rclcpp::ok();

  }
}

void IotBot::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) 
{
  // Assignment of joystick axes to motor commands
  float fwd      = joy->axes[1];            // Range of values [-1:1]
  float left     = joy->axes[0];            // Range of values [-1:1]
  float turn     = joy->axes[3];            // Range of values [-1:1]
  //float throttle = (joy->axes[7]+1.0)/2.0;  // Range of values [0:1]
  float throttle = 0,5; // 

  float vFwd  = throttle * fwd  * _vMax;
  float vLeft = throttle * left * _vMax;
  float omega = throttle * turn * _omegaMax;

  controlMotors(vFwd, vLeft, omega);
}

void IotBot::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd) 
{
   controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

void IotBot::controlMotors(float vFwd, float vLeft, float omega)
{
  
  float rpmFwd   = vFwd  * _ms2rpm;
  float rpmLeft  = vLeft * _ms2rpm;
  float rpmOmega = omega * _rad2rpm;
  rpmLeft = 0; // deactivated movement in y-direction
  
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
      {
        rpmMax = std::abs(_rpm[i]);        
      }
  }
  

  if(rpmMax > _motorParams->rpmMax)
  {
    float factor = _motorParams->rpmMax / rpmMax;
    for(int i=0; i<4; i++)
      _rpm[i] *= factor;
  }
  

  //_lastCmd = ros::Time::now();
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  _lastCmd = ros_clock.now();

}
