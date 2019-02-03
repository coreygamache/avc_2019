//manual control node
//this node handles manual control of the robot when autonomous control is disabled
#include <errno.h>
#include <ros/ros.h>
#include <avc_msgs/ChangeControlMode.h>
#include <avc_msgs/Control.h>
#include <avc_msgs/ESC.h>
#include <avc_msgs/SteeringServo.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>

//global variables
bool autonomous_control = false;

//global controller variables
std::vector<float> controller_axes(8, 0);
std::vector<int> controller_buttons(13, 0);


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const avc_msgs::Control::ConstPtr& msg)
{

  //verify that local mode matches global mode
  if (autonomous_control != msg->autonomous_control)
  {

    //modes do not match; send notification and shut down node
    ROS_INFO("[manual_control_mode] local control mode does not match global control mode; killing program");
    ROS_BREAK();

  }

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_axes = msg->axes;
  controller_buttons = msg->buttons;

}

//callback function called to process service requests on the disable mapping mode topic
bool disableManualControlCallback(avc_msgs::ChangeControlMode::Request& req, avc_msgs::ChangeControlMode::Response& res)
{

  //if node isn't currently busy then ready to change modes, otherwise not ready to change
  res.ready_to_change = true;

  //output ROS INFO message to inform of mode change request and reply status
  if (req.mode_change_requested && res.ready_to_change)
  {

    //change modes
    autonomous_control = !autonomous_control;

    //output notification
    ROS_INFO("[manual_control_mode] mode change requested; changing control modes");

  }
  else if (!req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[manual_control_mode] ready to change modes status requested; indicating ready to change");
  else if (req.mode_change_requested && !res.ready_to_change)
    ROS_INFO("[manual_control_mode] mode change requested; indicating node is busy");
  else
    ROS_INFO("[manual_control_mode] ready to change modes status requested; indicating node is busy");

  //return true to indicate service processing is complete
  return true;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting manual_control_mode");

  //initialize node and create node handler
  ros::init(argc, argv, "manual_control_mode");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve steering servo max rotation angle from parameter server
  float servo_max_angle;
  if (!node_private.getParam("/steering_servo/max_rotation_angle", servo_max_angle))
  {
    ROS_ERROR("[manual_control_node] steering servo max rotation angle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/manual_control_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[manual_control_node] manual control node refresh rate not defined in config file: avc_control/config/control.yaml");
    ROS_BREAK();
  }

  //create ESC message object and set default parameters
  avc_msgs::ESC esc_msg;
  esc_msg.header.frame_id = "0";

  //create steering servo message object and set default parameters
  avc_msgs::SteeringServo steering_servo_msg;
  steering_servo_msg.header.frame_id = "0";

  //create publisher to publish ESC message status with buffer size 10, and latch set to false
  ros::Publisher esc_pub = node_public.advertise<avc_msgs::ESC>("esc", 10, false);

  //create publisher to publish steering servo message status with buffer size 10, and latch set to false
  ros::Publisher steering_servo_pub = node_public.advertise<avc_msgs::SteeringServo>("steering_servo", 10, false);

  //create service to process service requests on the disable manual control topic
  ros::ServiceServer disable_manual_control_srv = node_public.advertiseService("disable_manual_control", disableManualControlCallback);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("control", 1000, controlCallback);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("joy", 1000, controllerCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    if (!autonomous_control)
    {

      //ESC control handling
      //set time of ESC message
      esc_msg.header.stamp = ros::Time::now();

      //translate controller axis value to percentage and set to esc msg value
      esc_msg.throttle_percent = ((controller_axes[5] - 1) / -2) * 100;

      //publish drive motors message
      esc_pub.publish(esc_msg);

      //steering servo control handling
      //set time of steering servo message
      steering_servo_msg.header.stamp = ros::Time::now();

      //translate controller axis value to percentage and set to esc msg value
      steering_servo_msg.steering_angle = controller_axes[0] * servo_max_angle;

      //publish drive motors message
      steering_servo_pub.publish(steering_servo_msg);

    }

  }
  return 0;
}
