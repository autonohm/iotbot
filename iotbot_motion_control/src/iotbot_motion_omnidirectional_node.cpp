#include "iotbot_motion_control.hpp"


namespace iotbot {


/**
 * @class CMotionOmnidirectionalNode
 * @brief Chield class for specific motion control
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 17.06.2021
 */
class CMotionOmnidirectionalNode : public CMotionControl
{
public:
    /**
     * constructor
     */
    CMotionOmnidirectionalNode()
    {
        velSub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                                       10,
                                                                       std::bind(&CMotionOmnidirectionalNode::velocityCallback,
                                                                       this,
                                                                       std::placeholders::_1));  
    }


    /**
     * destructor
     */
    ~CMotionOmnidirectionalNode() = default;
    
private:
    /**
     * callback function for the subscription of the velocity message
     * @param[in] cmd specifies the linear and angular parameters
     */
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd) override
    {     
        calculateKinematic(cmd);
        normalizeValues(); 
        publishRpm();
    }


    /**
     * calculate the specific kinematics of the used omnidirectional wheels
     * @param[in] cmd specifies the linear and angular parameters
     */
    void calculateKinematic(const geometry_msgs::msg::Twist::SharedPtr cmd) override
    {
        // for mecanum wheel mode  
        rpm_[chassisParams_.chFrontLeft]  = 10 * (( 2.0 / chassisParams_.wheelDiameter) * (   cmd->linear.x - cmd->linear.y )) - (conversionParams_.rad2rpm * cmd->angular.z);
        rpm_[chassisParams_.chFrontRight] = 10 * (( 2.0 / chassisParams_.wheelDiameter) * ( - cmd->linear.x - cmd->linear.y )) - (conversionParams_.rad2rpm * cmd->angular.z);
        rpm_[chassisParams_.chRearLeft]   = 10 * (( 2.0 / chassisParams_.wheelDiameter) * (   cmd->linear.x + cmd->linear.y )) - (conversionParams_.rad2rpm * cmd->angular.z);
        rpm_[chassisParams_.chRearRight]  = 10 * (( 2.0 / chassisParams_.wheelDiameter) * ( - cmd->linear.x + cmd->linear.y )) - (conversionParams_.rad2rpm * cmd->angular.z);  
    }
}; // CMotionOmnidirectionalNode


} // namespace iotbot



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iotbot::CMotionOmnidirectionalNode>());
  rclcpp::shutdown();

  return 0;
}
