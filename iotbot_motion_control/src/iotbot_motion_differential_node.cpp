#include "iotbot_motion_control.hpp"


namespace iotbot {


/**
 * @class CMotionDifferentialNode
 * @brief Chield class for specific motion control
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 17.06.2021
 */
class CMotionDifferentialNode : public CMotionControl
{
public:
    /**
     * constructor
     */
    CMotionDifferentialNode()
    {
        velSub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                                       10,
                                                                       std::bind(&CMotionDifferentialNode::velocityCallback,
                                                                       this,
                                                                       std::placeholders::_1));  
    }


    /**
     * destructor
     */
    ~CMotionDifferentialNode() = default;
    
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
     * calculate the specific kinematics of the used differential wheels
     * @param[in] cmd specifies the linear and angular parameters
     */
    void calculateKinematic(const geometry_msgs::msg::Twist::SharedPtr cmd) override
    {
        // For differantial mode
        float rpmFwd   = cmd->linear.x * conversionParams_.ms2rpm;
        float rpmLeft  = cmd->linear.y * conversionParams_.ms2rpm;
        float rpmOmega = cmd->angular.z * conversionParams_.rad2rpm;       
        
        rpmLeft = 0; // deactivated movement in y-direction
        
        // leading signs -> see derivation: Stefan May, Skriptum "Mobile Robotik"
        rpm_[chassisParams_.chFrontLeft]  =  rpmFwd - rpmLeft - rpmOmega;
        rpm_[chassisParams_.chFrontRight] = -rpmFwd - rpmLeft - rpmOmega;
        rpm_[chassisParams_.chRearLeft]   =  rpmFwd + rpmLeft - rpmOmega;
        rpm_[chassisParams_.chRearRight]  = -rpmFwd + rpmLeft - rpmOmega;
    }
}; // CMotionDifferentialNode


} // namespace iotbot



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iotbot::CMotionDifferentialNode>());
  rclcpp::shutdown();

  return 0;
}
