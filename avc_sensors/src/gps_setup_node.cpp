//ROS includes
#include <ros/ros.h>
#include <signal.h>
#include <string.h>
#include <wiringSerial.h>

//defines
#define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F\r\n"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C\r\n"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"

//global variables
int fd = -1;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //if serial device has successfully been opened then close it
  if (fd != -1)
    serialClose(fd);

  //call the default shutdown function
  ros::shutdown();

}

//function for sending messages to GPS via serial protocol
bool sendCommand(const char command_str[])
{

  //return false if provided file descriptor is invalid
  if (fd == -1)
    return false;

  //inform of command being sent to GPS chip
  ROS_INFO("[gps_setup_node] sending command to GPS chip: %s", command_str);

  //flush serial buffer before sending command
  serialFlush(fd);

  //output received command to serial buffer
  serialPrintf(fd, command_str);

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

  //-----------------------------SETUP WIRINGPI---------------------------------
/*
  if (wiringPiSetup() == -1)
  {
    ROS_INFO("[gps_setup_node] wiringPi setup failed");
    ROS_BREAK();
  }
*/
  //---------------------------OPEN SERIAL DEVICE-------------------------------

  //open serial device with default baud rate
  fd = serialOpen(serial_port.c_str(), baud_rate);

  //check if serial open succeeded
  if (fd == -1)
  {
    ROS_ERROR("[gps_setup_node] failed to open serial device");
    ROS_BREAK();
  }
  else
    ROS_INFO("[gps_setup_node] opened serial device %s with baud rate %d", serial_port.c_str(), baud_rate);

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //----------------------------------------------------------------------------
  //-----------------------SEND COMMANDS TO GPS CHIP----------------------------
  //----------------------------------------------------------------------------

  //-----------------------SET CHIP DATA OUTPUT TYPE----------------------------

  //set GPS chip data output type to RMC only
  if (!sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA))
    ROS_BREAK();

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //-----------------------SET CHIP DATA OUTPUT RATE----------------------------

  //set GPS chip data output rate to 5 Hz
  if (!sendCommand(PMTK_SET_NMEA_UPDATE_10HZ))
    ROS_BREAK();

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //-----------------------SET CHIP FIX UPDATE RATE-----------------------------

  //set GPS chip fix update rate to 5 Hz
  if (!sendCommand(PMTK_API_SET_FIX_CTL_5HZ))
    ROS_BREAK();

  //sleep briefly before running command
  ros::Duration(3.0).sleep();

  //--------------------------SET CHIP BAUD RATE--------------------------------

  //set GPS chip baud rate to 57600
  if (!sendCommand(PMTK_SET_BAUD_57600))
    ROS_BREAK();

  //sleep briefly before closing serial device
  ros::Duration(3.0).sleep();

  //----------------------------------------------------------------------------
  //-----------------------END COMMANDS TO GPS CHIP-----------------------------
  //----------------------------------------------------------------------------

  //close serial device
  serialClose(fd);

  //inform of serial device closure
  ROS_INFO("[gps_setup_node] closed serial device %s", serial_port.c_str());

  //sleep briefly before reopening serial device
  ros::Duration(1.0).sleep();

  //if program is still running at this point then everything has completed successfully
  //indicate that program has completed setup successfully
  ROS_INFO("[gps_setup_node] GPS chip setup completed successfully");

  return 0;
}



/*-----open serial device to confirm baud rate change------
//open serial device with new baud rate
fd = serialOpen(serial_port.c_str(), 57600);

//check if serial open succeeded
if (fd == -1)
{
  ROS_ERROR("[gps_setup_node] failed to open serial device");
  ROS_BREAK();
}
else
  ROS_INFO("[gps_setup_node] opened serial device %s with baud rate 57600", serial_port.c_str());

//sleep briefly before running command
ros::Duration(3.0).sleep();

//close serial device
serialClose(fd);*/



/*-----------------------------REPLY HANDLING---------------------------------

ROS_INFO("[gps_setup_node] command sent");

//sleep briefly while waiting for reply
ros::Duration(0.5).sleep();

//check for reply from GPS
if (serialDataAvail(fd) > 0)
{

  ROS_INFO("[gps_setup_node] reply received: %d", serialDataAvail(fd));

  //create vector container for reply
  std::vector<char> gps_reply;

  //receive reply character by character
  while (serialDataAvail > 0)
  {
    gps_reply.push_back(serialGetchar(fd));
    ROS_INFO("[gps_setup_node] character read: %c", gps_reply.back());
  }

  //create string to output reply contents
  std::string gps_reply_str(gps_reply.begin(), gps_reply.end());

  //inform of reply contents
  ROS_INFO("[gps_setup_node] reply received from GPS chip: %s", gps_reply_str.c_str());

  //if GPS chip acknowledgement packet indicates success then return true
  if ((gps_reply.size() > 13) && (gps_reply[13] == '3'))
  {

    //inform of success
    ROS_INFO("[gps_setup_node] valid command / packet, action succeeded");

    //return true to indicate success
    return true;

  }
  //in all other situtations command has failed
  else if ((gps_reply.size() > 13) && (gps_reply[13] == '2'))
    ROS_INFO("[gps_setup_node] valid command / packet, but action failed");
  else if ((gps_reply.size() > 13) && (gps_reply[13] == '1'))
    ROS_INFO("[gps_setup_node] unsupported command / packet type");
  else if ((gps_reply.size() > 13) && (gps_reply[13] == '0'))
    ROS_INFO("[gps_setup_node] invalid command / packet");
  else
    ROS_INFO("[gps_setup_node] invalid reply received from GPS");

}
else if (serialDataAvail(fd) == 0)
  ROS_INFO("[gps_setup_node] no reply received from GPS chip");
else if (serialDataAvail(fd) == -1)
  ROS_INFO("[gps_setup_node] attempt to check for available serial data failed: %s", strerror(errno));

//return false to indicate failure
return false;

-----------------------------END REPLY HANDLING-----------------------------*/
