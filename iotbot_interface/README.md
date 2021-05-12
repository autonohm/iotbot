# ROS2 package for IOTbot-related custom messages

#How to use these msgs
1. the first time you have to build the msgs
```bash
    cd /path/to/your/ros_ws
    colcon build --packages-select iotbot_interface
```

-> 

2. include the msgs
```C++
#include "iotbot_interface/msg/rotation_speed.hpp"
```

3. create a subscriber and callback for the unique msg 
```C++
rclcpp::Subscription<iotbot_interface::msg::RotationSpeed>::SharedPtr rpmSub_;
rpmSub_ = this->create_subscription<iotbot_interface::msg::RotationSpeed>("iotbot/rpm", 10, std::bind(&IotBotShield::rpmCallback, this, std::placeholders::_1));

void rpmCallback(const iotbot_interface::msg::RotationSpeed::SharedPtr  rpmMsg)
{
     last_time_= rclcpp::Clock().now();
     rpm[0] = rpmMsg->lf_rpm);
     rpm[1] = rpmMsg->rf_rpm)
     rpm[2] = rpmMsg->rr_rpm)
     rpm[3] = rpmMsg->lr_rpm)	  

}
```
