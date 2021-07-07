#include <functional>
#include <memory>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"

#include "iotbot_shield.hpp"
#include "iotbot_interface/msg/rotation_speed.hpp"
#include "iotbot_interface/msg/battery.hpp"


namespace iotbot {


enum EWheelPosition
{
    frontLeft   = 0,
    frontRight  = 1,
    rearLeft    = 2,
    rearRight   = 3
};


/**
 * @class CShieldNode
 * @brief Shield node class for connecting to the motor shield
 * @author David Grenner, Antonello Pastore, Christoph Kögel, Manuel Kreutz
 * @date 17.06.2021
 */
class CShieldNode : public rclcpp::Node
{
public:
    /**
     * constructor
     */
    CShieldNode() :
        Node("iotbot_shield"),
        currentTime_(rclcpp::Clock().now()),
        lastTime_(rclcpp::Clock().now()),
        deltaTime_(0)
    {
        rpmSubscriber_ = this->create_subscription<iotbot_interface::msg::RotationSpeed>("iotbot/rpm",
                                                                                         10,
                                                                                         std::bind(&CShieldNode::rpmCallback,
                                                                                         this,
                                                                                         std::placeholders::_1));

        rpmPublisher_ = this->create_publisher<iotbot_interface::msg::RotationSpeed>("iotbot/rpm/return",
                                                                                     10);

        batteryPublisher_ = this->create_publisher<iotbot_interface::msg::Battery>("iotbot/battery",
                                                                                   10); 
    }


    /**
     * destructor
     */
    ~CShieldNode()
    {
        sendLighting(iotbot::CMD_LIGHTS_PULSATION, 0, 0, 0);
        sendDisable();
    }


    /**
     * the basic while loop of the shield node
     */
    void run()
    {
        if (false == init())
        {
            return;
        }

        while (rclcpp::ok())
        {
            checkIfRpmMsgIsNew();

            publishRpm();
            publishVoltage();
        }
    }

private:
    static constexpr unsigned int MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE = 100;

    rclcpp::Subscription<iotbot_interface::msg::RotationSpeed>::SharedPtr   rpmSubscriber_;
    rclcpp::Publisher<iotbot_interface::msg::RotationSpeed>::SharedPtr      rpmPublisher_;
    rclcpp::Publisher<iotbot_interface::msg::Battery>::SharedPtr            batteryPublisher_;

    iotbot_interface::msg::RotationSpeed    rpmMessage_;
    iotbot_interface::msg::Battery          batteryMessage_;

    rclcpp::Time        currentTime_;
    rclcpp::Time        lastTime_;
    rclcpp::Duration    deltaTime_;

    CShield             serialPort_;
    SSerialRequestData  serialSendData_;
    SSerialResponseData serialReceiveData_;

    std::array<float, 4> rpm_;

private:
    /**
     * initialize UART and shield module with specific values for hardware setup
     */
    bool init()
    {
        if (false == serialPort_.openPort("/iotbot_ws/src/iotbot/iotbot_shield/serial_port_params.txt")) // or e.g. -> serialPort_.openPort("/dev/ttyS1", 115200)
        {
            RCLCPP_ERROR(this->get_logger(), "init() -> exiting because of an error with serial openPort()");
            return false;
        }
        
        sendEnable(); 
        sendGearRatio(51.f);
        sendTicksPerRev(4096.f);
        sendKp(0.f);
        sendKi(20.f);
        sendKd(0.f);
        sendLighting(iotbot::CMD_LIGHTS_DIM_LIGHT, 0, 0, 0);

        return true;
    }


    /**
     * check whether the subscribed RPM message is new or not and send current RPM to shield module
     */
    void checkIfRpmMsgIsNew()
    {
        currentTime_ = rclcpp::Clock().now();
        deltaTime_   = lastTime_ - currentTime_;

        if (deltaTime_.seconds() > 0.5f)
        {
            RCLCPP_INFO(this->get_logger(), "checkIfRpmMsgIsNew() -> time difference since last RPM message greater than 0.5 sec -> motor control will be deactivated");

            for (auto & it : rpm_)
            {
                it = 0.0;
            }

            sendRpm();
            sendDisable();
        }
        else
        {
            sendRpm();
        }
    }


    // ********************************************************************************* callback functions *********************************************************************************

    /**
     * the subscription callback function of the RPM message
     * @param[in] rpmMsg specifies the RPM parameters
     */
    void rpmCallback(const iotbot_interface::msg::RotationSpeed::SharedPtr rpmMsg)
    {
        lastTime_ = rclcpp::Clock().now();

        rpm_[EWheelPosition::frontLeft]  = rpmMsg->front_left_rpm;
        rpm_[EWheelPosition::frontRight] = rpmMsg->front_right_rpm;
        rpm_[EWheelPosition::rearLeft]   = rpmMsg->rear_left_rpm;
        rpm_[EWheelPosition::rearRight]  = rpmMsg->rear_right_rpm;
    }


    // ********************************************************************************* publish functions *********************************************************************************

    /**
     * publish the newly calculated RPM data to the network
     */
    void publishRpm()
    {
        rpmMessage_.front_left_rpm  = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::frontLeft])  / UINT16_TO_FLOAT_DIVISOR;
        rpmMessage_.front_right_rpm = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::frontRight]) / UINT16_TO_FLOAT_DIVISOR;
        rpmMessage_.rear_left_rpm   = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::rearLeft])   / UINT16_TO_FLOAT_DIVISOR;
        rpmMessage_.rear_right_rpm  = static_cast<float>(serialReceiveData_.rpm[EWheelPosition::rearRight])  / UINT16_TO_FLOAT_DIVISOR;

        rpmPublisher_->publish(rpmMessage_);
    }


    /**
     * publish the actual voltage to the network
     */
    void publishVoltage()
    {
        batteryMessage_.voltage = static_cast<float>(serialReceiveData_.voltage) / UINT16_TO_FLOAT_DIVISOR;

        batteryPublisher_->publish(batteryMessage_);
    }


    // ********************************************************************************* send functions *********************************************************************************

    /**
     * send enable command to shield module
     */
    void sendEnable()
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_ENABLE;

        for (auto & it : serialSendData_.payload)
        {
            it = 0;
        }

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendEnable() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send disable command to shield module
     */
    void sendDisable()
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_DISABLE;

        for (auto & it : serialSendData_.payload)
        {
            it = 0;
        }

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendDisable() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send gear ratio command to shield module
     * @param[in] gearRatio specifies the ratio of the used gear
     */
    void sendGearRatio(const float gearRatio)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_GEAR_RATIO;

        serialSendData_.payload[0] = static_cast<uint16_t>(gearRatio * FLOAT_TO_UINT16_MULTIPLIER);
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendGearRatio() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send ticks per revision command to shield module
     * @param[in] ticksPerRev specifies the resolution of the used encoder
     */
    void sendTicksPerRev(const float ticksPerRev)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_TICKS_PER_REV;

        serialSendData_.payload[0] = static_cast<uint16_t>(ticksPerRev * FLOAT_TO_UINT16_MULTIPLIER);
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendTicksPerRev() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send P regulator command to shield module
     * @param[in] kP specifies the P component of the regulator
     */
    void sendKp(const float kP)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_CTL_KP;

        serialSendData_.payload[0] = static_cast<uint16_t>(kP * FLOAT_TO_UINT16_MULTIPLIER);
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendKp() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send I regulator command to shield module
     * @param[in] kI specifies the I component of the regulator
     */
    void sendKi(const float kI)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_CTL_KI;

        serialSendData_.payload[0] = static_cast<uint16_t>(kI * FLOAT_TO_UINT16_MULTIPLIER);
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendKi() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send D regulator command to shield module
     * @param[in] kD specifies the D component of the regulator
     */
    void sendKd(const float kD)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_CTL_KD;

        serialSendData_.payload[0] = static_cast<uint16_t>(kD * FLOAT_TO_UINT16_MULTIPLIER);
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendKd() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send PWM command to shield module
     * @param[in] pwm specifies the pulse width modulation for each light
     */
    void sendPwm(const std::array<uint16_t, 4> & pwm)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_SET_PWM;

        for (std::size_t i = 0; i < pwm.size(); i++)
        {
            serialSendData_.payload[i] = pwm[i];
        }

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendPwm() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send LED lightning command with color settings to shield module
     * @param[in] command specifies the lightning command for the LEDs -> starts with CMD_LIGHTS_...
     * @param[in] rgbRed specifies the RGB color red (0...255)
     * @param[in] rgbGreen specifies the RGB color green (0...255)
     * @param[in] rgbBlue specifies the RGB color blue (0...255)
     */
    void sendLighting(const ESerialRequest command, const uint8_t rgbRed, const uint8_t rgbGreen, const uint8_t rgbBlue)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = command;

        serialSendData_.payload[0] = static_cast<uint16_t>(rgbRed);
        serialSendData_.payload[1] = static_cast<uint16_t>(rgbGreen);
        serialSendData_.payload[2] = static_cast<uint16_t>(rgbBlue);
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendLighting() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send AUX1 command to shield module
     * @param[in] on set to true to set AUX1 on, false to set it off
     */
    void sendAUX1(const bool on)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_AUX1;

        serialSendData_.payload[0] = on ? 1 : 0;
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendAUX1() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send AUX2 command to shield module
     * @param[in] on set to true to set AUX2 on, false to set it off
     */
    void sendAUX2(const bool on)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_AUX2;

        serialSendData_.payload[0] = on ? 1 : 0;
        serialSendData_.payload[1] = 0;
        serialSendData_.payload[2] = 0;
        serialSendData_.payload[3] = 0;

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendAUX2() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send RPM command to shield module
     * @param[in] rpm specifies the velocity goal for each wheel
     */
    void sendRpm(const std::array<float, 4> & rpm)
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_SET_RPM;

        for (std::size_t i = 0; i < rpm.size(); i++)
        {
            serialSendData_.payload[i] = static_cast<uint16_t>(rpm[i] * FLOAT_TO_UINT16_MULTIPLIER);
        }

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendRpm() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }


    /**
     * send RPM command to shield module (uses the internal RPM member array)
     */
    void sendRpm()
    {
        serialSendData_.start   = SERIAL_REQUEST_START_VALUE;
        serialSendData_.command = ESerialRequest::CMD_SET_RPM;

        for (std::size_t i = 0; i < rpm_.size(); i++)
        {
            serialSendData_.payload[i] = static_cast<uint16_t>(rpm_[i] * FLOAT_TO_UINT16_MULTIPLIER);
        }

        if (false == serialPort_.send(serialSendData_)) return;
        if (false == serialPort_.receive(serialReceiveData_, MAX_TIMEOUT_IN_MS_FOR_UART_RECEIVE)) return;

        if (ESerialResponse::CMD_SUCCESS != serialReceiveData_.command)
        {
            RCLCPP_WARN(this->get_logger(), "sendRpm() -> failure during send and receive with return command: 0x%x", serialReceiveData_.command);
        }
    }
}; // CShieldNode


} // namespace iotbot



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    iotbot::CShieldNode shieldNode;
    shieldNode.run();

    rclcpp::shutdown();

    return 0;
}
