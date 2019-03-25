//ESC control node
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <avc_msgs/ESC.h>
#include <signal.h>

//global variables
bool force_output = false;
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

//callback function to process timer firing event
void timerCallback(const ros::TimerEvent& event)
{

  //set force output to true to force servo output
  force_output = true;

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

  //retrieve ESC max pulsewidth value from parameter server [us]
  int esc_max_value;
  if (!node_private.getParam("/hardware/esc_node/esc_max_value", esc_max_value))
  {
    ROS_ERROR("[esc_node] ESC max value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve ESC min pulsewidth value from parameter server [us]
  int esc_min_value;
  if (!node_private.getParam("/hardware/esc_node/esc_min_value", esc_min_value))
  {
    ROS_ERROR("[esc_node] ESC min value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve ESC neutral pulsewidth value from parameter server [us]
  if (!node_private.getParam("/hardware/esc_node/esc_neutral_value", esc_neutral_value))
  {
    ROS_ERROR("[esc_node] ESC neutral value not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve force servo output time value from parameter server
  float force_output_time;
  if (!node_private.getParam("/hardware/servoblaster/force_output_time", force_output_time))
  {
    ROS_ERROR("[steering_servo_node] force servo output time not defined in config file: avc_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve max acceleration value from parameter server
  float maximum_acceleration;
  if (!node_private.getParam("/driving/maximum_acceleration", maximum_acceleration))
  {
    ROS_ERROR("[navigation_node] ESC maximum acceleration not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve max acceleration value from parameter server
  float maximum_deceleration;
  if (!node_private.getParam("/driving/maximum_deceleration", maximum_deceleration))
  {
    ROS_ERROR("[navigation_node] ESC maximum deceleration not defined in config file: avc_bringup/config/global.yaml");
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

  //create subscriber to subscribe to ESC message topic with queue size set to 1
  ros::Subscriber esc_sub = node_public.subscribe("esc", 1, escCallback);

  //calculate throttle forward and reverse ranges
  int esc_fwd_range = esc_max_value - esc_neutral_value;
  int esc_rev_range = esc_neutral_value - esc_min_value;


  //----- ESC ARMING SEQUENCE -----

  //inform of start of arming sequence
  ROS_INFO("[esc_node] beginning ESC arming sequence");

  //open servo driver
  std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

  for (int i = 0; i < 6; i++)
  {

    //output ESC neutral signal to servo driver
    sb_driver << esc_servo_number << "=" << (esc_neutral_value / 10) << "\n";

    //delay until next output to ESC
    ros::Duration(0.5).sleep();

  }

  //close file
  sb_driver.close();

  //inform of start of arming sequence
  ROS_INFO("[esc_node] ESC arming sequence complete");

  //----- END ESC ARMING SEQUENCE -----

  //create variable for remembering last throttle value
  float last_throttle_value = 0;

  //create timer object to be used to force servo output every set interval
  ros::Timer timer;

  //create timer to force servo write every predetermined interval
  timer = node_private.createTimer(ros::Duration(force_output_time), timerCallback, true);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if new throttle value was requested then output new value to ESC
    if ((throttle_percent != last_throttle_value) || force_output)
    {

      //if force output flag is true then reset to false
      if (force_output)
      {

        //reset force output flag to false
        force_output = false;

        //reset timer
        timer = node_private.createTimer(ros::Duration(force_output_time), timerCallback, true);

      }

      if (throttle_percent > 0)
      {
        if ((throttle_percent - last_throttle_value) > (maximum_acceleration / refresh_rate))
          throttle_percent = last_throttle_value + (maximum_acceleration / refresh_rate);
        else if ((last_throttle_value - throttle_percent)  > (maximum_deceleration / refresh_rate))
          throttle_percent = last_throttle_value - (maximum_deceleration / refresh_rate);
      }
      else if (throttle_percent < 0)
      {

      }


      //create pulsewidth variable to output calculated pulsewidth to esc
      int pulsewidth;

      //convert throttle value to pulsewidth [us / 10]
      if (throttle_percent >= 0)
        pulsewidth = esc_neutral_value + (throttle_percent / 100 * esc_fwd_range);
      else
        pulsewidth = esc_neutral_value - (fabs(throttle_percent) / 100 * esc_rev_range);

      //divide pulsewidth by 10 because servoblaster uses units of tens of microseconds
      pulsewidth = pulsewidth / 10;

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
