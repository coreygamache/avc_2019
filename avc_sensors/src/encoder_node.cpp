//ROS includes
#include <ros/ros.h>
#include <avc_msgs/Encoder.h>
#include <wiringPi.h>

//global variables
//unsigned int encoder_samples[ENCODER_SAMPLE_NUM];
//volatile unsigned int encoder_sample = 0;
//volatile unsigned int last_pulse_time = 0;
volatile unsigned int encoder_pulses = 0;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

void encoderInterruptCallback()
{

  //capture current pulse time, set sample time to time since current and last pulse, then update last pulse time
  /*unsigned int current_pulse_time = millis();
  encoder_sample = current_pulse_time - last_pulse_time;
  last_pulse_time = current_pulse_time;*/
  if (encoder_pulses < 4294967295)
    encoder_pulses++;
  else
    encoder_pulses = 0;

}

//sample velocity ENCODER_SAMPLE_NUM times and return average result
float getVelocity(int cpr, int sample_num, int refresh_rate)
{


  //calculate average sample time over last sample_num samples
  int num_pulses = 0;
  float start_time = millis();
  for (int i = 0; i < sample_num; i++)
  {
    num_pulses += encoder_pulses;
    encoder_pulses = 0;
    delay((1000 / refresh_rate) / (sample_num * 1.25));
  }

  //calculate velocity in rotations per second from sample time and counts per rev
  float velocity = num_pulses / ((millis() - start_time) / 1000); // [pulses/s]
  velocity =  (velocity / cpr) * 2 * 3.14159265359; // [rad/s]

  //return angular velocity in radians per second
  return velocity;

}

//CPU intensive option
/*void encoderInterruptCallback()
{

  //capture current pulse time
  unsigned int current_pulse_time = millis();

  //shift existing samples in array
  for (int i = 0; i < (ENCODER_SAMPLE_NUM - 1); i++)
  {
    encoder_samples[i+1] = encoder_samples[i];
  }

  //set newest encoder sample if this isn't the first pulse received
  if (last_pulse_time != 0)
    encoder_samples[0] = current_pulse_time - last_pulse_time;

  //set last pulse time to current pulse time for next iteration
  last_pulse_time = current_pulse_time;

}*/

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting encoder_pub_node");

  //initialize node and create node handler
  ros::init(argc, argv, "encoder_pub_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //if correct number of arguments has been passed to node, capture arguments and convert to strings
  std::string position_arg;
  if (argc == 3)
  {
    position_arg = boost::lexical_cast<std::string>(argv[1]) + boost::lexical_cast<std::string>(argv[2]);
  }
  else
  {
    ROS_ERROR("[ERROR] encoder_pub_node received incorrect number of arguments");
    ROS_BREAK();
  }

  //set encoder path for retrieving parameters from parameter server for this node
  std::string encoder_name = position_arg + "_encoder";
  std::string encoder_path = "/sensor/encoder/" + position_arg;
  ROS_INFO("encoder name: %s", encoder_name.c_str());
  ROS_INFO("encoder path: %s", encoder_path.c_str());

  //retrieve input pin parameter from parameter server
  int counts_per_rev;
  if (!node_private.getParam(encoder_path + "/counts_per_rev", counts_per_rev))
  {
    ROS_ERROR("counts per rev not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //retrieve input pin parameter from parameter server
  int input_pin;
  if (!node_private.getParam(encoder_path + "/input_pin", input_pin))
  {
    ROS_ERROR("encoder input pin not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from prameter server
  float refresh_rate;
  if (!node_private.getParam(encoder_path + "/refresh_rate", refresh_rate))
  {
    ROS_ERROR("encoder refresh rate not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //retrieve input pin parameter from parameter server
  int sample_num;
  if (!node_private.getParam(encoder_path + "/sample_num", sample_num))
  {
    ROS_ERROR("number of samples to use not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //call wiringPi setup function
  wiringPiSetup();

  //set pinMode of input pin to INPUT
  pinMode(input_pin, INPUT);

  //register interrupt function to be called when sensor input pin rising edge goes HIGH
  wiringPiISR(input_pin, INT_EDGE_RISING, encoderInterruptCallback);

  //create sensor_msgs/Range type message to publish proximity sensor data
  avc_msgs::Encoder encoder_msg;

  //create publisher to publish proximity sensor message with buffer size 10, and latch set to false
  ros::Publisher encoder_pub = node_public.advertise<avc_msgs::Encoder>(encoder_name, 10, false);

  //set refresh rate of ROS loop to defined refresh rate of sensor parameter
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set encoder message angular velocity value to current value reported from encoder [rad/s]
    encoder_msg.angular_velocity = getVelocity(counts_per_rev, sample_num, refresh_rate);

    //add ROS_INFO output to display current proximity sensor range to terminal (for testing)
    //ROS_INFO("current angular velocity: %f", encoder_msg.angular_velocity);

    //publish proximity sensor range message
    encoder_pub.publish(encoder_msg);

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
