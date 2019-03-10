# MTSU SME AVC Team Repository for 2019
## Overview ##
This is the public repository for the MTSU SME AVC team's robot for competition in the 2019 National Robotics Championship (NRC) Autonomous Vehicle Challenge (AVC). The goal of the Autonomous Vehicle Challenge is to autonomously navigate a provided course as quickly as possible, while avoiding certain obstacles and traversing others for bonus points. Competition rules constrain robot size and prohibit any outside communication with the robot with the exception of GPS signals.
## The Project ##
This robot was built on very limited budget, utilizing off-the-shelf RC car parts and hobby-level electronics and sensors. It relies primarily on GPS for navigation, but features a number of additional sensors to improve localization, allow for obstacle avoidance, etc.<br><br>
The robot has two control modes: manual and autonomous control. In manual control mode, the robot is controlled via BlueTooth with a controller. It can be driven like a typical RC car, and also has the ability to mark and save GPS waypoints. In autonomous mode the robot autonomously drives to the previously saved waypoints in the order they were entered. The hardware used is listed below, with the exception of typical electrical components (voltage regulators, resistors, etc.).
__Chassis__: Tamiya "The Hornet" RC Car<br>
__ESC__: HobbyWing QuicRun WP 1060 Brushed
__Steering Servo__: Futaba S0003<br>
__Microprocessors__: Raspberry Pi 3 Model B, Arduino Nano<br>
__GPS__: Adafruit Ultimate GPS with Active Antenna<br>
__IMU__: MPU-9250<br>
__Proximity Sensors__: HC-SR04 Ultrasonic Sensor (3)<br>
__Controller__: Sony DualShock 4 Wireless Controller<br>
## The Software ##
The robot's navigation software is built using ROS and coded in C++. It makes use of several widely available software and ROS packages, such as WiringPi, Joy for controller support and nmea_navsat_driver for GPS integration.<br><br>
__Robot Operating System (ROS)__: http://www.ros.org/<br>
__WiringPi__: http://wiringpi.com/<br>
__nmea_navsat_driver__: http://wiki.ros.org/nmea_navsat_driver<br>
__RTIMULib2__: https://github.com/Nick-Currawong/RTIMULib2<br><br>
__*Note*__: All software is compiled run on Ubuntu MATE 16.04 with ROS Kinetic on Raspberry Pi. Requires C++ 11 to compile. Special thanks to Stack Exchange for holding the answer to every programming question imaginable.<br>
## Setup ##
1. Build the robot.<br>
2. Install Ubuntu MATE 16.04, ROS Kinetic, WiringPi, catkin-tools, RTIMUImuLib2 on your Raspberry Pi.<br>
3. Enable serial, i2c, and SPI in raspi-config (DO NOT enable console via serial).<br>
4. Install nmea_navsat_driver and Joy ROS packages.<br>
5. Clone this repository into a new workspace and build it.<br>
6. Calibrate the IMU using RTIMULibCal and copy RTIMULib.ini to /avc_sensors/config/ (the provided calibration file will likely not work with your IMU).<br>
## Running ##
1. Navigate to the workspace you created previously.<br>
2. Run *source devel/setup.bash*<br>
3. Configure GPS settings by running *roslaunch avc_sensors gps_setup.launch*<br>
4. Launch the main software package with *roslaunch avc_bringup bringup.launch*<br>
5. Mark waypoints in manual mode: the robot can be driven using the left stick to steer and R2 for forward throttle or L2 for reverse throttle. GPS waypoints are set by pressing the X button, and the GPS waypoint list is saved by pressing triangle. Once saved, switch to autonomous mode by pressing the PS button.<br>
6. Press X in autonomous mode to begin autonomous navigation. When the route is complete you must switch to manual mode, where you may set a new list of waypoints. Alternatively, you can switch immediately back to autonomous mode and run the previous list of waypoints again. The list will be saved indefinitely until it is overwritten with a new list.<br><br>
