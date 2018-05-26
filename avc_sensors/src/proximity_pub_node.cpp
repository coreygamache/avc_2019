//ROS includes
#include "ros/ros.h"
#include "sensor_msgs/Range.h"

//other includes
#include "proximity_sensor.hpp"

int main(int argc, char const **argv)
{

  //initialize node and create node handler
  ros::init(argc, argc, "proximity_pub_node");
  ros::NodeHandle node_private("~");

  //create Sensor type object with echo set to GPIO pin 23 and trigger to 24
  ProximitySensor sensor(23, 24);

  //create sensor_msgs/Range type message to publish proximity sensor data
  sensor_msgs::Range proximity_msg;

  //static message values for HC-SR04 ultrasonic range sensor
  //----------------------------------------------------------

  //set sensor frame id
  proximity_msg.header.frame_id = "proximity_sensor_link";

  //set radiation type of sensor (0 indicates ultrasound, 1 infrared)
  proximity_msg.radiation_type = 0

  //set field of view of sensor in radians
  proximity_msg.field_of_view = 0.3665;

  //set minimum range of sensor in meters
  proximity_msg.min_range = 0.05;

  //set maximum range of sensor in meters
  proximity_msg.max_range = 2.0;

  //----------------------------------------------------------

  //create publisher to publish proximity sensor message with buffer size 10, and latch set to false
  ros::Publisher proximity_pub = node_private.advertise<sensor_msgs::Range>("proximity", 10, false);

  //set refresh rate to 10 hz
  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    //set time of current distance reading
    proximity_msg.header.stamp = ros::Time::now();

    //get distance to nearest object from proximity sensor with 25ms timeout]
    proximity_msg.range = sensor.getDistance();

    //publish proximity sensor range message
    proximity_pub.publish(proximity_msg);

    //spin once because ROS
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
