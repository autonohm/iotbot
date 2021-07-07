#ifndef IOTBOT_MOTION_CONTROL_H
#define IOTBOT_MOTION_CONTROL_H

#include <functional>
#include <memory>
#include <string>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "iotbot_interface/msg/rotation_speed.hpp"
#include "iotbot_robot_params.h"


namespace iotbot {


/**
 * @class CMotionControl
 * @brief Basic class for motion control
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 15.04.2021
 */
class CMotionControl : public rclcpp::Node
{
public:
    /**
     * constructor
     */
    CMotionControl();

    /**
     * destructor
     */
    ~CMotionControl() = default;

protected:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSub_; 
    SChassisParams chassisParams_;
    SRobotConversionParams conversionParams_;
    std::array<float, NUMBER_OF_WHEELS> rpm_;

protected:
    /**
     * set the RPM values to normalized values
     */
    void normalizeValues();

    /**
     * publish the newly calculated RPM data
     */
    void publishRpm();

    /**
     * callback function for the subscription of the velocity message -> has to be implemented in the child class
     * @param[in] cmd specifies the linear and angular parameters
     */
    virtual void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd) = 0;

    /**
     * calculate the specific kinematics of the used wheels -> has to be implemented in the child class
     * @param[in] cmd specifies the linear and angular parameters
     */
    virtual void calculateKinematic(const geometry_msgs::msg::Twist::SharedPtr cmd) = 0;

private:
    rclcpp::Publisher<iotbot_interface::msg::RotationSpeed>::SharedPtr rpmPub_;
    SMotorParams motorParams_;
    
private:
    /**
     * set specific motor parameters
     */
    void setMotorParams();
    
    /**
     * set specific chassis parameters
     */
    void setChassisParams();

    /**
     * calculate some additionally needed parameters with the specified motor & chassis parameters
     */
    void calculateRobotProperties();
};


} // namespace iotbot


#endif // IOTBOT_MOTION_CONTROL_H
