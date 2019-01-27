//map waypoints node
//this node controls mapping mode, which allows for mapping of GPS waypoints
//while mapping mode is enabled the robot can be manually controlled
#include <errno.h>
#include <ros/ros.h>
#include <avc_msgs/Control.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool mapping = false;
bool mapping_mode = false;
int map_waypoint_delay = 500; //time to wait for new GPS position data [ms]

//global controller variables
std::vector<int> controller_buttons(13, 0);

//global GPS position variables
std::vector<double> gpsFix(2, 0); //[latitude, longitude]
std::vector< std::vector<double> > gpsWaypoints;

//pin variables
//must be global so that they can be accessed by callback function
int indicator_LED;

//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //set all pins LOW
  digitalWrite(indicator_LED, LOW);

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const avc_msgs::Control::ConstPtr& msg)
{

  //set local value to match message value
  mapping_mode = !msg->autonomous_control;

  //inform of mode change
  if (mapping_mode)
    ROS_INFO("[map_waypoints_node] entering mapping mode");
  else
    ROS_INFO("[map_waypoints_node] exiting mapping mode");

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_buttons = msg->buttons;

  //if save waypoint button on controller is pressed, mapping mode is enabled, and not currently saving a waypoint then save waypoint
  if (mapping_mode && !mapping && (controller_buttons[0] == 1))
  {

    //set mapping variable to true to indicate waypoint is being saved
    mapping = true;

    //set button status to 0 to prevent consecutive toggles for one button press
    controller_buttons[0] = 0;

    //turn on LED and output text to indicate waypoint is being recorded
    digitalWrite(indicator_LED, HIGH);
    ROS_INFO("[map_waypoints_node] saving current position to list of waypoints");

    //wait momentarily for new GPS position signal
    delay(map_waypoint_delay);

    //process callback function calls
    ros::spinOnce();

    //add most recent GPS position to list of waypoints
    gpsWaypoints.push_back(gpsFix);

    //turn off LED and output text to indicate waypoint has been recorded
    digitalWrite(indicator_LED, LOW);
    ROS_INFO("[map_waypoints_node] waypoint saved (%d total waypoints)", int(gpsWaypoints.size()));

    //set mapping variable to false to indicate waypoint saving is complete
    mapping = false;

  }
  //if save list button on controller is pressed, mapping mode is enabled, and not currently saving a waypoint then save list
  //else is used to prevent attempting to save waypoint and waypoint list if buttons are pressed at same time
  else if (mapping_mode && !mapping && (controller_buttons[0] == 1))
  {

    //set mapping variable to true to indicate waypoint list is being saved
    mapping = true;

    //output list to csv file

    //flash LED three times to indicate waypoint list was saved
    for (int i = 0; i < 3; i++)
    {

      //flash LED
      digitalWrite(indicator_LED, HIGH);
      delay(250);
      digitalWrite(indicator_LED, LOW);

      //delay between flashes
      if (i < 2)
        delay(250);

    }

    //output text to indicate waypoint list was saved
    ROS_INFO("[map_waypoints_node] waypoint list saved to %s (%d total waypoints)", "file_path", int(gpsWaypoints.size()));

    //set mapping variable to false to indicate waypoint list saving is complete
    mapping = false;

  }

}

//callback function called to process messages on GPS fix topic
void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  //set local variables to match value received in message
  gpsFix[0] = msg->latitude;
  gpsFix[1] = msg->longitude;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting map_waypoints_node");

  //initialize node and create node handler
  ros::init(argc, argv, "map_waypoints_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve indicator LED from parameter server
  if (!node_private.getParam("/led/indicator_pin", indicator_LED))
  {
    ROS_ERROR("[map_waypoints_node] indicator LED pin not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("control", 1000, controlCallback);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("joy", 1000, controllerCallback);

  //create subscriber to subscribe to conveyor motor messages message topic with queue size set to 1000
  ros::Subscriber gps_fix_sub = node_public.subscribe("fix", 1000, gpsFixCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(indicator_LED, OUTPUT);

  //process callback function calls until node is shutdown
  ros::spin();

  return 0;
}
