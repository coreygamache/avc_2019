//ESC control node
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <avc_msgs/ESC.h>
#include <signal.h>

//global variables
float throttle_percent = 0;
int esc_neutral_value;
int esc_servo_number;
std::string sb_driver_path;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //open servo driver
  std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

  //output neutral value command to servo driver
  sb_driver << esc_servo_number << "=" << esc_neutral_value << "\n";

  //close file
  sb_driver.close();

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on drive_motors topic
void escCallback(const avc_msgs::ESC::ConstPtr& msg)
{

  //set local value to received value
  throttle_percent = msg->throttle_percent;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting esc_node");

  //initialize node and create node handler
  ros::init(argc, argv, "esc_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  int esc_max_value;
  if (!node_private.getParam("/hardware/esc_node/esc_max_value", esc_max_value))
  {
    ROS_ERROR("[esc_node] ESC max value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  int esc_min_value;
  if (!node_private.getParam("/hardware/esc_node/esc_min_value", esc_min_value))
  {
    ROS_ERROR("[esc_node] ESC min value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  if (!node_private.getParam("/hardware/esc_node/esc_neutral_value", esc_neutral_value))
  {
    ROS_ERROR("[esc_node] ESC neutral value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/esc_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[esc_node] ESC node refresh rate not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servoblaster driver path from parameter server
  if (!node_private.getParam("/hardware/servoblaster/sb_driver_path", sb_driver_path))
  {
    ROS_ERROR("[esc_node] servoblaster driver path not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servoblaster servo number from parameter server
  if (!node_private.getParam("/hardware/servoblaster/esc_servo_number", esc_servo_number))
  {
    ROS_ERROR("[esc_node] ESC servo number not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //divide esc values by 10 to match units used by servoblaster driver
  esc_max_value = esc_max_value / 10;
  esc_min_value = esc_min_value / 10;
  esc_neutral_value = esc_neutral_value / 10;

  //create subscriber to subscribe to ESC message topic with queue size set to 1000
  ros::Subscriber esc_sub = node_public.subscribe("esc", 1000, escCallback);

  //calculate throttle forward and reverse ranges
  int esc_fwd_range = esc_max_value - esc_neutral_value;
  int esc_rev_range = esc_neutral_value - esc_min_value;

  //create variable for remembering last throttle value
  float last_throttle_value = 9999;

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if new throttle value was requested then output new value to ESC
    if (throttle_percent != last_throttle_value)
    {

      //create pulsewidth variable to output calculated pulsewidth to esc
      int pulsewidth;

      //convert throttle value to pulsewidth [us / 10]
      if (throttle_percent >= 0)
        pulsewidth = esc_neutral_value + int(throttle_percent / 100 * esc_fwd_range);
      else
        pulsewidth = esc_neutral_value - int(abs(throttle_percent) / 100 * esc_rev_range);

      //open servo driver
      std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

      //output throttle change command to servo driver
      sb_driver << esc_servo_number << "=" << pulsewidth << "\n";

      //close file
      sb_driver.close();

      //set last throttle value to current throttle value
      last_throttle_value = throttle_percent;

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
