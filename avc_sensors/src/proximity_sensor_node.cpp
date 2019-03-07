//ROS includes
#include <proximity_sensor.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <signal.h>

//macro definitions for median filter
#define swap(a,b) a^=b; b^=a; a^=b;
#define sort(a,b) if(a>b) { swap(a,b); }


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

int medianFilter(int lastReadings[])
{

    //perform optimal combination of sorts to produce sorted array
    sort(lastReadings[0], lastReadings[1]);
    sort(lastReadings[3], lastReadings[4]);
    sort(lastReadings[0], lastReadings[2]);
    sort(lastReadings[1], lastReadings[2]);
    sort(lastReadings[0], lastReadings[3]);
    sort(lastReadings[2], lastReadings[3]);
    sort(lastReadings[1], lastReadings[4]);
    sort(lastReadings[1], lastReadings[2]);

    //return middle value of sorted array (the median value)
    return lastReadings[2];

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting proximity_sensor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "proximity_sensor_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //get echo pin from parameters
  int echo_pin;
  if (!node_private.getParam("/proximity_sensor/front/echo_pin", echo_pin))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor echo pin not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //get trigger pin from parameters
  int trigger_pin;
  if (!node_private.getParam("/proximity_sensor/front/trigger_pin", trigger_pin))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor trigger pin not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //create Sensor type object using defined echo and trigger pin parameters
  ProximitySensor sensor(echo_pin, trigger_pin);

  //create sensor_msgs/Range type message to publish proximity sensor data
  sensor_msgs::Range proximity_msg;

  //static message values for HC-SR04 ultrasonic range sensor
  //----------------------------------------------------------

  //set sensor frame id
  proximity_msg.header.frame_id = "proximity_sensor_link";

  //set radiation type of sensor (0 indicates ultrasound, 1 infrared)
  int radiation_type;
  if (!node_private.getParam("/sensor/proximity_sensor/radiation_type", radiation_type))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor radiation type not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }
  proximity_msg.radiation_type = radiation_type;

  //set field of view of sensor in radians
  float field_of_view;
  if (!node_private.getParam("/sensor/proximity_sensor/field_of_view", field_of_view))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor field of view not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }
  proximity_msg.field_of_view = field_of_view;

  //set minimum range of sensor in meters
  float min_range;
  if (!node_private.getParam("/sensor/proximity_sensor/min_range", min_range))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor minimum range not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }
  proximity_msg.min_range = min_range;

  //set maximum range of sensor in meters
  float max_range;
  if (!node_private.getParam("/sensor/proximity_sensor/max_range", max_range))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor maximum range not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }
  proximity_msg.max_range = max_range;

  //----------------------------------------------------------

  //create publisher to publish proximity sensor message with buffer size 10, and latch set to false
  ros::Publisher proximity_pub = node_public.advertise<sensor_msgs::Range>("proximity", 10, false);

  //get refresh rate of sensor in hertz
  float refresh_rate;
  if (!node_private.getParam("/sensor/proximity_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[proximity_sensor_node] sensor refresh rate not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //create buffer for storing five most recent readings to be used by median filter
  int numReadings = 0;
  int lastReadings[5] = { -1, -1, -1, -1, -1 };

  //set refresh rate of ROS loop to defined refresh rate of sensor parameter
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set time of current distance reading
    proximity_msg.header.stamp = ros::Time::now();

    //get distance to nearest object from proximity sensor with 25ms timeout [mm]
    lastReadings[numReadings++ % 5] = sensor.getDistance(25) * 1000;

    //set message range value to median filtered sensor reading [m]
    proximity_msg.range = float(medianFilter(lastReadings)) / 1000;

    //publish proximity sensor range message
    proximity_pub.publish(proximity_msg);

    //prevent overflow of int type variable
    if (numReadings == 2147483647)
      numReadings = 0;

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
