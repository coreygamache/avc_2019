//ROS includes
#include <proximity_sensor.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <signal.h>
#include <string.h>

//macro definitions for median filter (sorting method)
#define swap(a,b) a^=b; b^=a; a^=b;
#define sort(a,b) if(a>b) { swap(a,b); }

//macro definitions for median filter (algorithm method)
#define STOPPER 0 // smaller than any datum
#define MEDIAN_FILTER_SIZE 7


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//median filter using fancy algorithm (more efficient for filter sizes > 3)
int medianFilter(int datum)
{

  struct pair
  {
    struct pair *point;                                //Pointers forming list linked in sorted order
    uint16_t value;                                    //Values to sort
  };

  static struct pair buffer[MEDIAN_FILTER_SIZE] = {0}; //Buffer of nwidth pairs
  static struct pair *datpoint = buffer;               //Pointer into circular buffer of data
  static struct pair small = {NULL, STOPPER};          //Chain stopper
  static struct pair big = {&small, 0};                //Pointer to head (largest) of linked list

  struct pair *successor;                              //Pointer to successor of replaced data item
  struct pair *scan;                                   //Pointer used to scan down the sorted list
  struct pair *scanold;                                //Previous value of scan
  struct pair *median;                                 //Pointer to median

  if (datum == STOPPER)
    datum = STOPPER + 1;                               //No stoppers allowed

  if ((datpoint++ - buffer) >= MEDIAN_FILTER_SIZE)
    datpoint = buffer;                                 //Increment and wrap data in pointer

  datpoint->value = datum;                             //Copy in new datum
  successor = datpoint->point;                         //Save pointer to old value's successor
  median = &big;                                       //Median initially to first in chain
  scanold = NULL;                                      //Scanold initially null
  scan = &big;                                         //Points to pointer to first (largest) datum in chain

  //Handle chain-out of first item in chain as special case
  if (scan->point == datpoint)
    scan->point = successor;

  scanold = scan;                                      //Save this pointer and
  scan = scan->point ;                                 //step down chain

  //Loop through the chain, normal loop exit via break
  for (int i = 0 ; i < MEDIAN_FILTER_SIZE; i++)
  {

    //Handle odd-numbered item in chain
    if (scan->point == datpoint)
      scan->point = successor;                         //Chain out the old datum

    if (scan->value < datum)                           //If datum is larger than scanned value,
    {
      datpoint->point = scanold->point;                //Chain it in here
      scanold->point = datpoint;                       //Mark it chained in
      datum = STOPPER;
    }

    //Step median pointer down chain after doing odd-numbered element
    median = median->point;                            //Step median pointer
    if (scan == &small)
      break;                                           //Break at end of chain
    scanold = scan;                                    //Save this pointer and
    scan = scan->point;                                //step down chain

    //Handle even-numbered item in chain.
    if (scan->point == datpoint)
      scan->point = successor;

    if (scan->value < datum)
    {
      datpoint->point = scanold->point;
      scanold->point = datpoint;
      datum = STOPPER;
    }

    if (scan == &small)
      break;

    scanold = scan;
    scan = scan->point;
  }

 return median->value;

}

//median filter using simple sorting logic
int medianFilterSorted(int lastReadings[])
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

  //define encoder_number via passed argument
  /*char *sensor_position;
  if (argc == 2)
    sensor_position = argv[1];
  else
  {
    ROS_ERROR("[proximity_sensor_node] received incorrect number of arguments");
    ROS_BREAK();
  }*/

  //set node parameter path for retrieving parameters from parameter server for this node
  std::string parameter_path = "/proximity_sensor/front/";//"/proximity_sensor/" + boost::lexical_cast<std::string>(sensor_position) + "/";

  //get echo pin from parameters
  int echo_pin;
  if (!node_private.getParam(parameter_path + "echo_pin", echo_pin))
  {
    ROS_ERROR("[proximity_sensor_node] proximity sensor echo pin not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //get refresh rate of sensor in hertz
  float refresh_rate;
  if (!node_private.getParam("/sensor/proximity_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[proximity_sensor_node] sensor refresh rate not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //get trigger pin from parameters
  int trigger_pin;
  if (!node_private.getParam(parameter_path + "trigger_pin", trigger_pin))
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

  //create buffer for storing five most recent readings to be used by median filter
  //int numReadings = 0;
  //int lastReadings[5] = { -1, -1, -1, -1, -1 };

  //set refresh rate of ROS loop to defined refresh rate of sensor parameter
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set time of current distance reading
    proximity_msg.header.stamp = ros::Time::now();

    //get distance to nearest object from proximity sensor with 25ms timeout [m]
    //lastReadings[numReadings++ % 5] = sensor.getDistance(25) * 1000;
    proximity_msg.range = medianFilter(int(sensor.getDistance(25) * 1000)) / 1000;

    //set message range value to median filtered sensor reading [m]
    //proximity_msg.range = float(medianFilter(lastReadings)) / 1000;

    //verify distance from proximity sensor is valid
    //if distance check timed out then report max range
    if ((proximity_msg.range == -1) || (proximity_msg.range > max_range))
      proximity_msg.range = max_range;
    else if (proximity_msg.range < min_range)
      proximity_msg.range = min_range;

    //publish proximity sensor range message
    proximity_pub.publish(proximity_msg);

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
