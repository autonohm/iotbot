# ROS2 package iotbot
ROS2 interface to robotic extension shield for the IOT2050 device

# Quick Start
Setting UART parameters via console and start iotbot_node:
```console
root@iot2050-debian:~# stty -F /dev/ttyS1 115200 cs8 -cstopb -parenb
root@iot2050-debian:~# ros2 run iotbot2 iotbot_node
```
