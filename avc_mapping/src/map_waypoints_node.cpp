//map waypoints node
//this node controls mapping mode, which allows for mapping of GPS waypoints
//while mapping mode is enabled the robot can be manually controlled
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <errno.h>
#include <ros/ros.h>
#include <avc_msgs/ChangeControlMode.h>
#include <avc_msgs/Control.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <wiringPi.h>

//macro definition for averaging filter
#define LAST_FIXES 10 //number of GPS fixes to average

//global variables
bool mapping = false;
bool mapping_mode = true;

//global controller variables
std::vector<int> controller_buttons(13, 0);

//global GPS position variables
std::vector< std::vector<double> > gpsWaypoints;
std::vector< std::vector<double> > lastFixes(LAST_FIXES);

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

  //change local control mode to match message
  mapping_mode = !msg->autonomous_control;

  //clear list of GPS waypoints on mode change`
  if (!mapping_mode)
    gpsWaypoints.clear();

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_buttons = msg->buttons;

}

//callback function called to process service requests on the disable mapping mode topic
bool disableMappingCallback(avc_msgs::ChangeControlMode::Request& req, avc_msgs::ChangeControlMode::Response& res)
{

  //if node isn't currently busy then ready to change modes, otherwise not ready to change
  res.ready_to_change = !mapping;

  //output ROS INFO message to inform of mode change request and reply status
  if (req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[map_waypoints_node] mode change requested; changing control modes");
  else if (!req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[map_waypoints_node] ready to change modes status requested; indicating ready to change");
  else if (req.mode_change_requested && !res.ready_to_change)
    ROS_INFO("[map_waypoints_node] mode change requested; indicating node is busy");
  else
    ROS_INFO("[map_waypoints_node] ready to change modes status requested; indicating node is busy");

  //return true to indicate service processing is complete
  return true;

}

//callback function called to process messages on GPS fix topic
void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  //create counter for placing most recent fix in vector
  static int fixCounter = 0;

  //on first iteration populate entire vector with current GPS fix
  if (fixCounter == 0)
  {

    //iterate through each element of vector, inserting current GPS fix data
    for (int i = 0; i < LAST_FIXES; i++)
    {
      lastFixes[i][0] = msg->latitude;
      lastFixes[i][1] = msg->longitude;
    }

  }
  else
  {

    //put most recent GPS fix data into appropriate position in vector
    lastFixes[fixCounter % fixCounter][0] = msg->latitude;
    lastFixes[fixCounter % fixCounter][1] = msg->longitude;

  }

  //increment counter
  fixCounter++;

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

  //retrieve indicator LED pin from parameter server
  if (!node_private.getParam("/led/indicator_pin", indicator_LED))
  {
    ROS_ERROR("[map_waypoints_node] indicator LED pin not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve map waypoint delay from parameter server [ms]
  int map_waypoint_delay;
  if (!node_private.getParam("/mapping/map_waypoints_node/map_waypoint_delay", map_waypoint_delay))
  {
    ROS_ERROR("[map_waypoints_node] map waypoint delay not defined in config file: avc_mapping/config/mapping.yaml");
    ROS_BREAK();
  }

  //retrieve map waypoint delay from parameter server [ms]
  std::string output_file_path;
  if (!node_private.getParam("/mapping/output_file_path", output_file_path))
  {
    ROS_ERROR("[map_waypoints_node] GPS waypoint output file path not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/mapping/map_waypoints_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[map_waypoints_node] control node refresh rate not defined in config file: avc_mapping/config/mapping.yaml");
    ROS_BREAK();
  }

  //create service to process service requests on the disable mapping topic
  ros::ServiceServer disable_mapping_srv = node_public.advertiseService("/control/disable_mapping", disableMappingCallback);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("/control/control", 1000, controlCallback);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("/control/joy", 1000, controllerCallback);

  //create subscriber to subscribe to conveyor motor messages message topic with queue size set to 1000
  ros::Subscriber gps_fix_sub = node_public.subscribe("/sensor/fix", 1000, gpsFixCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(indicator_LED, OUTPUT);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

   //if save waypoint button on controller is pressed, mapping mode is enabled, and not currently saving a waypoint then save waypoint
   if (mapping_mode && (controller_buttons[1] == 1))
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

      //create vector to hold current GPS location
      std::vector<double> gpsFix(2, 0);

      //set gpsFix values to sum of specified number of last GPS fixes
      for (int i = 0; i < LAST_FIXES; i++)
      {
        gpsFix[0] += lastFixes[i][0];
        gpsFix[1] += lastFixes[i][1];
      }

      //divide values by number of last GPS fixes to calculate average values
      gpsFix[0] = gpsFix[0] / LAST_FIXES;
      gpsFix[1] = gpsFix[1] / LAST_FIXES;

      //add most recent GPS position to list of waypoints
      gpsWaypoints.push_back(gpsFix);

      //turn off LED and output text to indicate waypoint has been recorded
      ROS_INFO("[map_waypoints_node] waypoint saved (%d total waypoints)", int(gpsWaypoints.size()));
      ROS_INFO("[map_waypoints_node] coordinates: %lf, %lf", gpsFix[0], gpsFix[1]);
      digitalWrite(indicator_LED, LOW);

      //set mapping variable to false to indicate waypoint saving is complete
      mapping = false;

    }
    //if save list button on controller is pressed, mapping mode is enabled, and not currently saving a waypoint then save list
    //else is used to prevent attempting to save waypoint and waypoint list if buttons are pressed at same time
    else if (mapping_mode && (controller_buttons[3] == 1))
    {

      //output list to csv file
      //create file object and open file path
      std::fstream output_file(output_file_path.c_str(), std::fstream::out | std::fstream::trunc);

      //output header to top of file
      output_file << "latitude [deg]," << "longitude [deg]\n";

      //output each waypoint in list to file
      for (int i = 0; i < gpsWaypoints.size(); i++)
        output_file << std::fixed << std::setprecision(12) << gpsWaypoints[i][0] << "," << gpsWaypoints[i][1] << "\n";

      //close file after all waypoints have been output
      output_file.close();


      //output text to indicate waypoint list was saved
      ROS_INFO("[map_waypoints_node] waypoint list saved to %s (%d total waypoints)", output_file_path.c_str(), int(gpsWaypoints.size()));

      //flash LED twice to indicate waypoint list was saved
      for (int i = 0; i < 2; i++)
      {

        //flash LED
        digitalWrite(indicator_LED, HIGH);
        delay(500);
        digitalWrite(indicator_LED, LOW);

        //delay between flashes
        if (i < 1)
          delay(500);

      }

    }

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
