# ROS package iotbot
ROS interface to robotic extension shield for the IOT2050 device

# Quick Start
Setting UART parameters via console and start iotbot_node:
```console
root@iot2050-debian:~# stty -F /dev/ttyS1 115200 cs8 -cstopb -parenb
root@iot2050-debian:~# roscore &
root@iot2050-debian:~# cd catkin_ws
root@iot2050-debian:~/catkin_ws# ./devel/lib/iotbot/iotbot_node
```

Connect a USB joystick and start the proper ROS node, e.g.
```console
root@iot2050-debian:~/catkin_ws# rosrun joy joy_node joy:=/joy _autorepeat_rate:=100
```
