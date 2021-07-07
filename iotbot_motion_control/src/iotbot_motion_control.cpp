#include "iotbot_motion_control.hpp"


namespace iotbot {


CMotionControl::CMotionControl() : Node("iotbot_motion_control")    
{
    setMotorParams();
    setChassisParams();
    calculateRobotProperties();

    rpmPub_ = this->create_publisher<iotbot_interface::msg::RotationSpeed>("iotbot/rpm",
                                                                           10);         
}



void CMotionControl::setMotorParams()
{
    motorParams_.gearRatio      = 51.f;
    motorParams_.encoderRatio   = 4096.f;
    motorParams_.rpmMax         = 80.f;

    RCLCPP_INFO(this->get_logger(), "setMotorParams() -> gear ratio is set to:                       %.0f", motorParams_.gearRatio );
    RCLCPP_INFO(this->get_logger(), "setMotorParams() -> encoder ratio is set to:                    %.0f", motorParams_.encoderRatio);
    RCLCPP_INFO(this->get_logger(), "setMotorParams() -> the maximum speed of the motors are set to: %.2f rpm", motorParams_.rpmMax);
}



void CMotionControl::setChassisParams()
{
    chassisParams_.track            = 0.35f;
    chassisParams_.wheelBase        = 0.24f;
    chassisParams_.wheelDiameter    = 0.10f;

    chassisParams_.chFrontLeft      = 1; // TODO: stimmt die Reihenfolge (1, 2, 0, 3) ???
    chassisParams_.chFrontRight     = 2;
    chassisParams_.chRearLeft       = 0;
    chassisParams_.chRearRight      = 3;

    chassisParams_.direction        = 1;

    RCLCPP_INFO(this->get_logger(), "setChassisParams() -> track of the robot is set to:          %.2f m", chassisParams_.track);
    RCLCPP_INFO(this->get_logger(), "setChassisParams() -> wheelbase of the robot is set to:      %.2f m", chassisParams_.wheelBase );
    RCLCPP_INFO(this->get_logger(), "setChassisParams() -> wheel diameter of the robot is set to: %.3f m", chassisParams_.wheelDiameter);
}



void CMotionControl::calculateRobotProperties()
{
    // ensure that the direction parameter is set properly (either 1 or -1)
    if (0 < chassisParams_.direction)
    {
        chassisParams_.direction = 1;
    }
    else
    {
        chassisParams_.direction = -1;
    }

    conversionParams_.rad2rpm   = (chassisParams_.wheelBase + chassisParams_.track) / chassisParams_.wheelDiameter; // (lx+ly)/2 * 1/r
    conversionParams_.rpm2rad   = 1.f / conversionParams_.rad2rpm;

    conversionParams_.ms2rpm    = 60.f / (chassisParams_.wheelDiameter * M_PI);
    conversionParams_.rpm2ms    = 1.f / conversionParams_.ms2rpm;

    conversionParams_.vMax      = motorParams_.rpmMax * conversionParams_.rpm2ms;
    conversionParams_.omegaMax  = motorParams_.rpmMax * conversionParams_.rpm2rad;

    RCLCPP_INFO(this->get_logger(), "calculateRobotProperties() -> initialized IotBot with vMax:     %.2f m/s", conversionParams_.vMax );
    RCLCPP_INFO(this->get_logger(), "calculateRobotProperties() -> initialized IotBot with omegaMax: %.2f rad", conversionParams_.omegaMax); 
}
  


void CMotionControl::normalizeValues()
{
    float rpmCurrentMax = motorParams_.rpmMax;
    
    for (auto & rpmIterator : rpm_)
    {
        // possibility to flip directions
        rpmIterator *= chassisParams_.direction;
        
        // normalize values, if any value exceeds the maximum
        if (std::abs(rpmIterator) > motorParams_.rpmMax)
        {
            rpmCurrentMax = std::abs(rpmIterator);        
        }
    }

    if (rpmCurrentMax > motorParams_.rpmMax)
    {
        float factor = motorParams_.rpmMax / rpmCurrentMax;

        for (auto & rpmIterator : rpm_)
        {
            rpmIterator *= factor;
        }
    }
}



void CMotionControl::publishRpm()
{
    auto msgRpm = iotbot_interface::msg::RotationSpeed();

    msgRpm.front_left_rpm  = rpm_[EWheelPosition::frontLeft];
    msgRpm.front_right_rpm = rpm_[EWheelPosition::frontRight];
    msgRpm.rear_left_rpm   = rpm_[EWheelPosition::rearLeft];
    msgRpm.rear_right_rpm  = rpm_[EWheelPosition::rearRight];

    rpmPub_->publish(msgRpm);   
}


} // namespace iotbot
