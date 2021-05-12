#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "RobotParams.h"
#include "iotbot_interface/msg/rotation_speed.hpp"


// using namespace std::chrono_literals;
// using std::placeholders::_1;

class IotBotMotorControl : public rclcpp::Node
{
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSub_; 
  rclcpp::Publisher<iotbot_interface::msg::RotationSpeed>::SharedPtr rpmPub_;
  
  
  MotorParams             motorParams;
  ChassisParams           chassisParams;
  RobotConversionParams   conversionParams;

  float rpmBuffer[4] = {0.0, 0.0 , 0.0 , 0.0};
  float rpm[4] = {0.0, 0.0 , 0.0 , 0.0};

public:
  IotBotMotorControl()   : Node("iotbot_motion_control")       
  {
    loadMotorParams();
    loadChassisParams();
    calculateRobotProperties();


   // Create Publisher and Subscriber 
    velSub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&IotBotMotorControl::velocityCallback, this, std::placeholders::_1));          
    rpmPub_ = this->create_publisher<iotbot_interface::msg::RotationSpeed>("cmd/return", 10);         
  
  }

  ~IotBotMotorControl(){
   
    RCLCPP_INFO(this->get_logger(), "Closed iotbot_motion_control");
  }


private:

  void loadChassisParams(){
    chassisParams.track =               0.35f;
    chassisParams.wheelBase =           0.24f;
    chassisParams.wheelDiameter =       0.10f;
    
    chassisParams.chFrontLeft =         1;
    chassisParams.chFrontRight =        2;
    chassisParams.chRearLeft =          0;
    chassisParams.chRearRight =         3;

    chassisParams.direction =           1;
    
    RCLCPP_INFO(this->get_logger(), "The track of the Robot is set to: %.2f m", chassisParams.track);
    RCLCPP_INFO(this->get_logger(), "The wheelbase of the Robot is set to: %.2f m", chassisParams.wheelBase );
    RCLCPP_INFO(this->get_logger(), "The wheel diameter of the Robot is set to: %.3f m", chassisParams.wheelDiameter);
    
  }
  
  void loadMotorParams(){
    motorParams.gearRatio =             51.f;
    motorParams.encoderRatio =          4096.f;
    motorParams.rpmMax =                80.f;

    RCLCPP_INFO(this->get_logger(), "Gear Ratio is set to: %.0f ", motorParams.gearRatio );
    RCLCPP_INFO(this->get_logger(), "Encoder Ratio is set to: %.0f ", motorParams.encoderRatio);
    RCLCPP_INFO(this->get_logger(), "The maximum speed of the motors are set to:  %.2f rpm", motorParams.rpmMax);
  }

  void calculateRobotProperties(){
    
    // ensure that the direction parameter is set properly (either 1 or -1)
    if(chassisParams.direction>0) chassisParams.direction = 1;
    else chassisParams.direction = -1;

    conversionParams.rad2rpm          = (chassisParams.wheelBase  +   chassisParams.track)  / chassisParams.wheelDiameter; // (lx+ly)/2 * 1/r
    conversionParams.rpm2rad          = 1.0 / conversionParams.rad2rpm;
    
    conversionParams.ms2rpm           = 60.0 / (chassisParams.wheelDiameter * M_PI);
    conversionParams.rpm2ms           = 1.0 / conversionParams.ms2rpm;
    
    conversionParams.vMax             = motorParams.rpmMax * conversionParams.rpm2ms;
    conversionParams.omegaMax         = motorParams.rpmMax * conversionParams.rpm2rad;

    RCLCPP_INFO(this->get_logger(), "Initialized IotBot with vMax: %.2f m/s", conversionParams.vMax );
    RCLCPP_INFO(this->get_logger(), "Initialized IotBot with omegaMax: %.2f rad", conversionParams.omegaMax); 
    
  }
  
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd){     
    
    calculateRpm(cmd);
    publishData(); 
    normalizeValues(); 
    publishData();
    
    
  }

  void calculateRpm(const geometry_msgs::msg::Twist::SharedPtr cmd){
  
    // For differantial mode

    // float rpmFwd   = cmd->linear.x  * conversionParams.ms2rpm;
    // float rpmLeft  = cmd->linear.y  * conversionParams.ms2rpm;
    // float rpmOmega = cmd->angular.z  * conversionParams.rad2rpm;
    
    // rpmLeft = 0; // deactivated movement in y-direction
        
    // leading signs -> see derivation: Stefan May, Skriptum Mobile Robotik
    // rpmBuffer[chassisParams.chFrontLeft]  =  rpmFwd - rpmLeft - rpmOmega;
    // rpmBuffer[chassisParams.chFrontRight] = -rpmFwd - rpmLeft - rpmOmega;
    // rpmBuffer[chassisParams.chRearLeft]   =  rpmFwd + rpmLeft - rpmOmega;
    // rpmBuffer[chassisParams.chRearRight]  = -rpmFwd + rpmLeft - rpmOmega;
    

    // for mecanum wheel mode  
    rpmBuffer[chassisParams.chFrontLeft]  = 10 * (( 2.0 / chassisParams.wheelDiameter)  * (   cmd->linear.x - cmd->linear.y )) - (conversionParams.rad2rpm  * cmd->angular.z);
    rpmBuffer[chassisParams.chFrontRight] = 10 * (( 2.0 / chassisParams.wheelDiameter)  * ( - cmd->linear.x - cmd->linear.y )) - (conversionParams.rad2rpm  * cmd->angular.z);
    rpmBuffer[chassisParams.chRearLeft]   = 10 * (( 2.0 / chassisParams.wheelDiameter)  * (   cmd->linear.x + cmd->linear.y )) - (conversionParams.rad2rpm  * cmd->angular.z);
    rpmBuffer[chassisParams.chRearRight]  = 10 * (( 2.0 / chassisParams.wheelDiameter)  * ( - cmd->linear.x + cmd->linear.y )) - (conversionParams.rad2rpm  * cmd->angular.z);    

   
}

  void normalizeValues(){
  // possibility to flip directions
     for(int i=0; i<4; i++)
    {
      rpmBuffer[i] *= chassisParams.direction;
    }          

      // Normalize values, if any value exceeds the maximum
    float rpmMax = std::abs(rpmBuffer[0]);
    for(int i=1; i<4; i++)
    {
      // if(std::abs(rpm_[i]) > motorParams.rpmMax)
      if(std::abs(rpmBuffer[i]) > rpmMax)
        {
          rpmMax = std::abs(rpmBuffer[i]);        
        }
    }

    if(rpmMax > motorParams.rpmMax)
    {
      float factor = motorParams.rpmMax / rpmMax;
      for(int i=0; i<4; i++)
      {
       rpm[i] = rpmBuffer[i] * factor;
      }
    }
    else
    {
      for(int i=0; i<4; i++)
      {
        rpm[i] = rpmBuffer[i];
      }
    }
  }

  void publishData(){

    auto msgRPM = iotbot_interface::msg::RotationSpeed();
  

    msgRPM.front_left_rpm  = rpm[0];
    msgRPM.front_right_rpm = rpm[1];
    msgRPM.rear_left_rpm   = rpm[2];
    msgRPM.rear_right_rpm  = rpm[3];

    rpmPub_->publish(msgRPM);   
  }
}; // end of IotBotMotorControl class


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IotBotMotorControl>());
  rclcpp::shutdown();
  return 0;
}
