//subscribe to navigation output (heading error [rad]), odometry
//determine forward velocity as a function of steering angle, set as target
//compare current forward velocity to target and apply correction (PID control)
#include <errno.h>
#include <ros/ros.h>
#include <avc_msgs/Control.h>
#include <avc_msgs/ESC.h>
#include <avc_msgs/SteeringServo.h>
#include <signal.h>
#include <wiringPiI2C.h>

//global variables
bool autonomous_control = false;
float steering_angle = 0;
float throttle_percent = 0;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const avc_msgs::Control::ConstPtr& msg)
{

  //set local value to received value
  autonomous_control = msg->autonomous_control;

  //reset steering and throttle values on control mode change
  steering_angle = 0;
  throttle_percent = 0;

}

//callback function called to process messages on drive_motors topic
void escCallback(const avc_msgs::ESC::ConstPtr& msg)
{

  //set local value to received value
  throttle_percent = msg->throttle_percent;

}

void steeringServoCallback(const avc_msgs::SteeringServo::ConstPtr& msg)
{

  //set local value to received value
  steering_angle = msg->steering_angle;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting arduino_output_node");

  //initialize node and create node handler
  ros::init(argc, argv, "arduino_output_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve arduino i2c address from parameter server (global parameters)
  int i2c_address;
  if (!node_private.getParam("/arduino/i2c_address", i2c_address))
  {
    ROS_ERROR("[arduino_output_node] arduino i2c address not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/drive_motor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[arduino_output_node] arduino output node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to control message topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("/control/control", 1000, controlCallback);

  //create subscriber to subscribe to ESC message topic with queue size set to 1000
  ros::Subscriber esc_sub = node_public.subscribe("/hardware/esc", 1000, escCallback);

  //create subscriber to subscribe to steering servo message topic with queue size set to 1000
  ros::Subscriber steering_servo_sub = node_public.subscribe("/hardware/steering_servo", 1000, steeringServoCallback);

  //initialize i2c protocol and verify connection
  int fd = wiringPiI2CSetup(i2c_address);
  int result; //variable for holding i2c read/write result

  //output notification message and error if one occurs
  if (fd == -1)
    ROS_INFO("[arduino_output_node] error establishing i2c connection: %d", errno);
  else
    ROS_INFO("[arduino_output_node] i2c connection result: %d", fd);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //create array and set output values to current throttle percent and steering angle values
    unsigned char outputValues[2] = { int(throttle_percent), int(steering_angle) };

    //output motor PWM values to arduino via i2c protocol
    result = write(fd, outputValues, 2);

    //output notification message if error occurs
    if (result == -1)
    {
      ROS_INFO("[arduino_output_node] error writing to arduino via i2c: %d", errno);
    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
