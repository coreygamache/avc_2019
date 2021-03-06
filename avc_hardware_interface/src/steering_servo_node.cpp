//subscribe to navigation output (heading error [rad]), odometry
//determine forward velocity as a function of steering angle, set as target
//compare current forward velocity to target and apply correction (PID control)
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <avc_msgs/SteeringServo.h>
#include <signal.h>

//global variables
bool force_output = false;
float steering_angle = 0;
int ss_neutral_value;
int ss_servo_number;
std::string sb_driver_path;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //open servo driver
  std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

  //output neutral value command to servo driver
  sb_driver << ss_servo_number << "=" << ss_neutral_value << "\n";

  //close file
  sb_driver.close();

  //call the default shutdown function
  ros::shutdown();

}

void steeringServoCallback(const avc_msgs::SteeringServo::ConstPtr& msg)
{

  //set local value to received value
  steering_angle = msg->steering_angle;

}

//callback function to process timer firing event
void timerCallback(const ros::TimerEvent& event)
{

  //set force output to true to force servo output
  force_output = true;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting steering_servo_node");

  //initialize node and create node handler
  ros::init(argc, argv, "steering_servo_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve force servo output time value from parameter server
  float force_output_time;
  if (!node_private.getParam("/hardware/servoblaster/force_output_time", force_output_time))
  {
    ROS_ERROR("[steering_servo_node] force servo output time not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servo max rotation angle value from parameter server [deg]
  float ss_max_angle;
  if (!node_private.getParam("/steering_servo/max_rotation_angle", ss_max_angle))
  {
    ROS_ERROR("[steering_servo_node] steering servo max angle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve servo max right pulsewidth value from parameter server [us]
  int ss_max_right;
  if (!node_private.getParam("/hardware/steering_servo_node/ss_max_right", ss_max_right))
  {
    ROS_ERROR("[steering_servo_node] steering servo max value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servo max left pulsewidth value from parameter server [us]
  int ss_max_left;
  if (!node_private.getParam("/hardware/steering_servo_node/ss_max_left", ss_max_left))
  {
    ROS_ERROR("[steering_servo_node] steering servo min value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servo neutral pulsewidth value from parameter server [us]
  if (!node_private.getParam("/hardware/steering_servo_node/ss_neutral_value", ss_neutral_value))
  {
    ROS_ERROR("[steering_servo_node] steering servo neutral value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/steering_servo_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[steering_servo_node] arduino output node refresh rate not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servoblaster driver path from parameter server
  if (!node_private.getParam("/hardware/servoblaster/sb_driver_path", sb_driver_path))
  {
    ROS_ERROR("[steering_servo_node] servoblaster driver path not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servoblaster servo number from parameter server
  if (!node_private.getParam("/hardware/servoblaster/ss_servo_number", ss_servo_number))
  {
    ROS_ERROR("[steering_servo_node] steering servo number not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to steering servo message topic with queue size set to 1
  ros::Subscriber steering_servo_sub = node_public.subscribe("steering_servo", 1, steeringServoCallback);

  //calculate throttle forward and reverse ranges
  int ss_left_range = ss_neutral_value - ss_max_left;
  int ss_right_range = ss_max_right - ss_neutral_value;

  //create variable for remembering last throttle value
  float last_steering_value = 9999;

  //create timer object to be used to force servo output every set interval
  ros::Timer timer;

  //create timer to force servo write every predetermined interval
  timer = node_private.createTimer(ros::Duration(force_output_time), timerCallback, true);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if new throttle value was requested then output new value to ESC
    if ((steering_angle != last_steering_value) || force_output)
    {

      //if force output flag is true then reset to false
      if (force_output)
      {

        //reset force output flag to false
        force_output = false;

        //reset timer
        timer = node_private.createTimer(ros::Duration(force_output_time), timerCallback, true);

      }

      //create pulsewidth variable to output calculated pulsewidth to esc
      int pulsewidth;

      //convert throttle value to pulsewidth [us / 10]
      if (steering_angle >= 0)
        pulsewidth = ss_neutral_value - (steering_angle / ss_max_angle * ss_left_range);
      else
        pulsewidth = ss_neutral_value + (fabs(steering_angle) / ss_max_angle * ss_right_range);

      //divide pulsewidth by 10 because servoblaster uses units of tens of microseconds
      pulsewidth = pulsewidth / 10;

      //open servo driver
      std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

      //output throttle change command to servo driver
      sb_driver << ss_servo_number << "=" << pulsewidth << "\n";

      //close file
      sb_driver.close();

      //set last throttle value to current throttle value
      last_steering_value = steering_angle;

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
