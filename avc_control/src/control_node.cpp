//control node
//this node controls control mode changing via controller button press
#include <errno.h>
#include <ros/ros.h>
#include <avc_msgs/ChangeControlMode.h>
#include <avc_msgs/Control.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool autonomous_control = false;
bool mode_change_requested = false;

//global controller variables
std::vector<int> controller_buttons(13, 0);

//pin variables
//must be global so that they can be accessed by callback function
int mode_LED_blue_pin;
int mode_LED_green_pin;
int mode_LED_red_pin;

//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //set all pins LOW
  digitalWrite(mode_LED_blue_pin, LOW);
  digitalWrite(mode_LED_green_pin, LOW);
  digitalWrite(mode_LED_red_pin, LOW);

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_buttons = msg->buttons;

  //if mode change controller button is pressed then set change mode request true
  if (controller_buttons[12] == 1)
  {

    //set mode change requested to true to indicate request to change modes
    mode_change_requested = true;

    //inform of control made change request
    ROS_INFO("[control_node] control mode change requested");

    //reset controller button if pressed to prevent mode from toggling twice on one button press
    controller_buttons[12] = 0;

  }

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting control_node");

  //initialize node and create node handler
  ros::init(argc, argv, "control_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve mode LED pin (blue) from parameter server
  if (!node_private.getParam("/led/mode_blue_pin", mode_LED_blue_pin))
  {
    ROS_ERROR("[map_waypoints_node] mode LED pin (blue) not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve mode LED pin (green) from parameter server
  if (!node_private.getParam("/led/mode_green_pin", mode_LED_green_pin))
  {
    ROS_ERROR("[map_waypoints_node] mode LED pin (green) not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve mode LED pin (red) from parameter server
  if (!node_private.getParam("/led/mode_red_pin", mode_LED_red_pin))
  {
    ROS_ERROR("[map_waypoints_node] mode LED pin (red) not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/control_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[control_node] control node refresh rate not defined in config file: avc_control/config/control.yaml");
    ROS_BREAK();
  }

  //create control message object and set default parameters
  avc_msgs::Control control_msg;
  control_msg.header.frame_id = "0";

  //create disable manual control service object and set default parameters
  avc_msgs::ChangeControlMode disable_manual_control_srv;
  disable_manual_control_srv.request.mode_change_requested = true;

  //create disable mapping service object and set default parameters
  avc_msgs::ChangeControlMode disable_mapping_srv;
  disable_mapping_srv.request.mode_change_requested = true;

  //create disable navigation object and set default parameters
  avc_msgs::ChangeControlMode disable_navigation_srv;
  disable_navigation_srv.request.mode_change_requested = true;

  //create publisher to publish control message status with buffer size 10, and latch set to true
  ros::Publisher control_pub = node_public.advertise<avc_msgs::Control>("control", 10, true);

  //create service client to send service requests on the disable navigation topic
  ros::ServiceClient disable_manual_control_clt = node_public.serviceClient<avc_msgs::ChangeControlMode>("disable_manual_control");

  //create service client to send service requests on the disable mapping topic
  ros::ServiceClient disable_mapping_clt = node_public.serviceClient<avc_msgs::ChangeControlMode>("disable_mapping");

  //create service client to send service requests on the disable navigation topic
  ros::ServiceClient disable_navigation_clt = node_public.serviceClient<avc_msgs::ChangeControlMode>("disable_navigation");

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("joy", 1000, controllerCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(mode_LED_blue_pin, OUTPUT);
  pinMode(mode_LED_green_pin, OUTPUT);
  pinMode(mode_LED_red_pin, OUTPUT);

  //create variable for status of autonomous control
  bool autonomous_control = false;

  //create vectors for storing LED RGB values corresponding to each control mode (R, G, B)
  std::vector<int> led_autonomous_mode;
  led_autonomous_mode.push_back(0);
  led_autonomous_mode.push_back(0);
  led_autonomous_mode.push_back(1);
  std::vector<int> led_mapping_mode;
  led_mapping_mode.push_back(0);
  led_mapping_mode.push_back(1);
  led_mapping_mode.push_back(0);

  //initialize control message to disable autonomous control globally
  control_msg.header.stamp = ros::Time::now();
  control_msg.autonomous_control = false;

  //publish initial control message to disable autonomous control globally
  control_pub.publish(control_msg);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    if (mode_change_requested)
    {

      //check for verification from manual control program that it's safe to change modes
      if (!disable_manual_control_clt.call(disable_manual_control_srv))
      {
        ROS_INFO("[control_node] failed to call disable manual control service");
        ROS_BREAK();
      }

      //wait for verification from mapping program that it's safe to change modes
      if (!disable_mapping_clt.call(disable_mapping_srv))
      {
        ROS_INFO("[control_node] failed to call disable mapping service");
        ROS_BREAK();
      }

      //wait for verification from navigation program that it's safe to change modes
      if (!disable_navigation_clt.call(disable_navigation_srv))
      {
        ROS_INFO("[control_node] failed to call disable navigation service");
        ROS_BREAK();
      }

      //notify of any busy nodes
      if (!disable_manual_control_srv.response.ready_to_change)
        ROS_INFO("[control_node] manual control node not ready for mode change");
      if (!disable_mapping_srv.response.ready_to_change)
        ROS_INFO("[control_node] mapping node not ready for mode change");
      if (!disable_navigation_srv.response.ready_to_change)
        ROS_INFO("[control_node] navigation node not ready for mode change");

      //if mapping and navigation programs are ready to change control modes, then switch control modes and publish message
      if (disable_manual_control_srv.response.ready_to_change && disable_mapping_srv.response.ready_to_change && disable_navigation_srv.response.ready_to_change)
      {

        //reset mode change requested to prevent mode from toggling twice on one button press
        mode_change_requested = false;

        //switch control modes
        autonomous_control = !autonomous_control;

        //set time and parameters of control message
        control_msg.header.stamp = ros::Time::now();
        control_msg.autonomous_control = autonomous_control;

        //publish control message
        control_pub.publish(control_msg);

        //output result and change LED illumination to reflect current control mode
        if (autonomous_control)
        {
          ROS_INFO("[control_node] control mode changed: entering autonomous control mode");
          digitalWrite(mode_LED_red_pin, led_autonomous_mode[0]);
          digitalWrite(mode_LED_green_pin, led_autonomous_mode[1]);
          digitalWrite(mode_LED_blue_pin, led_autonomous_mode[2]);
        }
        else
        {
          ROS_INFO("[control_node] control mode changed: entering mapping control mode");
          digitalWrite(mode_LED_red_pin, led_mapping_mode[0]);
          digitalWrite(mode_LED_green_pin, led_mapping_mode[1]);
          digitalWrite(mode_LED_blue_pin, led_mapping_mode[2]);
        }

      }
      //if one more more nodes aren't ready for mode change then notify of failure and retry on next iteration
      else
        ROS_INFO("[control_node] one or more nodes not yet ready to change modes; retrying");

    }

  }

  return 0;
}
