//GPS setup node
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <ros/ros.h>
#include <signal.h>
#include <string.h>

//defines
#define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F\r\n"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C\r\n"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//function for sending messages to GPS via serial protocol
bool sendCommand(const char serial_port[], const char command_str[])
{

  //inform of command being sent to GPS chip
  ROS_INFO("[gps_setup_node] sending command to GPS chip: %s", command_str);

  std::fstream gps_output(serial_port, std::fstream::out | std::fstream::trunc);

  gps_output << command_str;

  gps_output.close();

  //return true to indicate command sent successfully
  return true;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting gps_setup_node");

  //initialize node and create node handler
  ros::init(argc, argv, "gps_setup_node");
  ros::NodeHandle node_private("~");

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve baud rate of sensor in hertz from parameter server
  int baud_rate;
  if (!node_private.getParam("/setup/baud_rate", baud_rate))
  {
    ROS_ERROR("[gps_setup_node] GPS chip baud rate not defined in config file: avc_sensors/config/gps.yaml");
    ROS_BREAK();
  }

  //retrieve serial port address from parameter server
  std::string serial_port;
  if (!node_private.getParam("/setup/serial_port", serial_port))
  {
    ROS_ERROR("[gps_setup_node] GPS chip serial port not defined in config file: avc_sensors/config/gps.yaml");
    ROS_BREAK();
  }

  //----------------------------------------------------------------------------
  //-----------------------SEND COMMANDS TO GPS CHIP----------------------------
  //----------------------------------------------------------------------------

  //-----------------------SET CHIP DATA OUTPUT TYPE----------------------------

  //set GPS chip data output type to RMC only
  if (!sendCommand(serial_port.c_str(), PMTK_SET_NMEA_OUTPUT_RMCGGA))
    ROS_BREAK();

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //-----------------------SET CHIP DATA OUTPUT RATE----------------------------

  //set GPS chip data output rate to 5 Hz
  if (!sendCommand(serial_port.c_str(), PMTK_SET_NMEA_UPDATE_10HZ))
    ROS_BREAK();

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //-----------------------SET CHIP FIX UPDATE RATE-----------------------------

  //set GPS chip fix update rate to 5 Hz
  if (!sendCommand(serial_port.c_str(), PMTK_API_SET_FIX_CTL_5HZ))
    ROS_BREAK();

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //--------------------------SET CHIP BAUD RATE--------------------------------

  //set GPS chip baud rate to 57600
  if (!sendCommand(serial_port.c_str(), PMTK_SET_BAUD_57600))
    ROS_BREAK();

  //sleep briefly before closing serial device
  ros::Duration(3.0).sleep();

  //----------------------------------------------------------------------------
  //-----------------------END COMMANDS TO GPS CHIP-----------------------------
  //----------------------------------------------------------------------------

  //if program is still running at this point then everything has completed successfully
  //indicate that program has completed setup successfully
  ROS_INFO("[gps_setup_node] GPS chip setup completed successfully");

  return 0;
}
