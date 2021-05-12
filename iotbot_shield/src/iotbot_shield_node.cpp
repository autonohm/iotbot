#include <functional>
#include <memory>

#include <string>

#include "rclcpp/rclcpp.hpp"

//include own library
#include "IOTShield.h"
#include "iotbot_msgs/msg/rotation_speed.hpp"
#include "iotbot_msgs/msg/battery.hpp"

// using namespace std::chrono_literals;
// using std::placeholders::_1;

class IotBotShield : public rclcpp::Node
{  
	rclcpp::TimerBase::SharedPtr timer_;
	
  	rclcpp::Subscription<iotbot_msgs::msg::RotationSpeed>::SharedPtr rpmSub_;
	rclcpp::Publisher<iotbot_msgs::msg::RotationSpeed>::SharedPtr rpmPub_;
	rclcpp::Publisher<iotbot_msgs::msg::Battery>::SharedPtr batteryPub_
  
  	rclcpp::Time      current_time_ = rclcpp::Clock().now();
  	rclcpp::Time      last_time_ = rclcpp::Clock().now();
  	rclcpp::Duration  delta_time_ = current_time_ - last_time_; 
	
	//Todo move this somewhere else
	const int frontLeft = 0;
	const int frontRight = 1;
	const int RearLeft = 2;
	const int RearRight = 3;

public:
	IotBotShield()   : Node("iotbot_motion_control"){
	   
		iotbot::IOTShield iotshield;
    	
   		rpmSub_ = this->create_subscription<iotbot_msgs::msg::RotationSpeed>("iotbot/rpm", 1, std::bind(&IotBotShield::rpmCallback, this, std::placeholders::_1));

		rpmPub_ = this->create_publisher<iotbot_msgs::msg::RotationSpeed>("iotbot/rpm/return", 10);
		batteryPub_ = this->create_publisher<iotbot_msgs::msg::Battery>("iotbot/battery", 1);
		 
		timer_ = this->create_wall_timer(500ms, std::bind(&IotBotShield::timerCallback, this));
		
  	}

private:

	void timerCallback(){
		
		checkRPMtimeout();

		publishReturnedRpmData();
		publishVoltage();

	}

	void rpmCallback(const iotbot_msgs::msg::RotationSpeed::SharedPtr  rpmMsg)
	{
		last_time_= rclcpp::Clock().now();
		
		float rpmBuffer[4];

		rpmBuffer[0] = rpmMsg->lf_rpm;
		rpmBuffer[1] = rpmMsg->rf_rpm;
		rpmBuffer[2] = rpmMsg->rr_rpm;
		rpmBuffer[3] = rpmMsg->lr_rpm;
		
		iotshield.setRPM(rpmBuffer)
	}

	void checkRPMtimeout(){

		current_time_ = rclcpp::Clock().now();
		delta_time_ = last_time_ - current_time_;
		if(delta_time_.seconds() > 0.5)
		{
			float rpmBuffer[4] = {0,0,0,0};
			iotshield.setRPM(rpmBuffer)
		}

	  }


	void publishReturnedRpmData(){

		//TODO: get Data from iotshield.
		auto msgRpmReturned = iotbot_msgs::msg::RotationSpeed();

		msgRpmReturned.lf_rpm = //TODO retuned arraqy from iotbotshield.
		msgRpmReturned.rf_rpm = //TODO retuned arraqy from iotbotshield.
		msgRpmReturned.rr_rpm = //TODO retuned arraqy from iotbotshield.
		msgRpmReturned.lr_rpm = //TODO retuned arraqy from iotbotshield.

		rpmPub_->publish(msgRpmReturned);

	}

	void publishVoltage(){
		
		auto msgBattery = iotbot_msgs::msg::Battery();
		msgBattery.voltage = iotshield.getSystemVoltage();

		batteryPub_->publish(msgBattery);
	}		

 
}; // end of IotBotShield classSS


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IotBotShield>());
  rclcpp::shutdown();
  return 0;
}
