# MTSU SME AVC Team Repository for 2019
## Overview ##
This is the public repository for the MTSU SME AVC team's robot for competition in the 2019 National Robotics Championship (NRC) Autonomous Vehicle Challenge (AVC). The goal of the Autonomous Vehicle Challenge is to autonomously navigate a provided course as quickly as possible, while avoiding certain obstacles and traversing others for bonus points. Competition rules constrain robot size and prohibit any outside communication with the robot with the exception of GPS signals.
## The Team ##
__Corey__: Team Captain, Navigation and Hardware Integration<br>
__Nathanael__: Simulation<br>
__Nicole__: PCB Design and Creation<br>
__Dustin__: Electrical and Mechanical Design<br>
__Piro__: MIA<br>
## The Robot ##
The robot uses a combination of microprocessors and sensors for autonomous navigation, listed below.<br><br>
__Chassis__: Tamiya "The Hornet" RC Car<br>
__Steering Servo__: Futaba S0003<br>
__Microprocessors__: Arduino Nano and Raspberry Pi 3 Model B<br>
__GPS__: Adafruit Ultimate GPS with Active Antenna<br>
__IMU__: MPU-9250<br>
__LiDAR__:<br>
__Proximity Sensors__: HC-SR04 Ultrasonic Sensor<br>
## The Software ##
The robot's navigation software is built using the ROS framework and coded primarily in C++. It is comprised of a number of ROS packages which, together, enable autonomous navigation by leaning heavily on a number of packages included in ROS, namely the navigation stack. In addition, a number of third party packages and libraries including nmea_navsat_driver and RTIMULib2 were used for hardware integration. We'd like to extend a big thank you to everyone involved in creating and maintaining the software and code that made this project possible. More information can be found by following the links below.<br><br>
__Robot Operating System (ROS)__: http://www.ros.org/<br>
__ROS Navigation Stack__: http://wiki.ros.org/navigation<br>
__nmea_navsat_driver__: http://wiki.ros.org/nmea_navsat_driver<br>
__RTIMULib2 (MPU-9255 fork)__: https://github.com/Nick-Currawong/RTIMULib2<br><br><br>
__*Note*__: C++ 11 is required to compile this repository.
