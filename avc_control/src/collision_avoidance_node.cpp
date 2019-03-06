//control node
//this node controls control mode changing via controller button press
#include <errno.h>
#include <ros/ros.h>
#include <avc_msgs/Control.h>
#include <avc_msgs/ESC.h>
#include <avc_msgs/SteeringServo.h>
#include <sensor_msgs/Range.h>
#include <signal.h>

//global variables
bool autonomous_control = false;
float range_to_nearest = -1; //range to nearest object [m]
float throttle_percent; //requested throttle position [%]
float steering_angle; //requested steering angle [deg]


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const avc_msgs::Control::ConstPtr& msg)
{

  //change local control mode to match message
  autonomous_control = msg->autonomous_control;

}

//callback function called to process messages on drive_motors topic
void escCallback(const avc_msgs::ESC::ConstPtr& msg)
{

  //set local value to received value
  throttle_percent = msg->throttle_percent;

}

//callback function called to process messages on control topic
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{

  //change local control mode to match message
  range_to_nearest = msg->range;

}

void steeringServoCallback(const avc_msgs::SteeringServo::ConstPtr& msg)
{

  //set local value to received value
  steering_angle = msg->steering_angle;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting collision_avoidance_node");

  //initialize node and create node handler
  ros::init(argc, argv, "collision_avoidance_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve danger distance threshold from parameter server
  float danger_threshold;
  if (!node_private.getParam("/control/collision_avoidance_node/danger_threshold", danger_threshold))
  {
    ROS_ERROR("[collision_avoidance_node] danger distance threshold not defined in config file: avc_control/config/control.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/collision_avoidance_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[collision_avoidance_node] collision avoidance node refresh rate not defined in config file: avc_control/config/control.yaml");
    ROS_BREAK();
  }

  //retrieve warning distance threshold from parameter server
  float warning_threshold;
  if (!node_private.getParam("/control/collision_avoidance_node/warning_threshold", warning_threshold))
  {
    ROS_ERROR("[collision_avoidance_node] warning distance threshold not defined in config file: avc_control/config/control.yaml");
    ROS_BREAK();
  }

  //create ESC message object and set default parameters
  avc_msgs::ESC esc_msg;
  esc_msg.header.frame_id = "0";

  //create steering servo message object and set default parameters
  avc_msgs::SteeringServo steering_servo_msg;
  steering_servo_msg.header.frame_id = "0";

  //create publisher to publish ESC message status with buffer size 10, and latch set to false
  ros::Publisher esc_pub = node_public.advertise<avc_msgs::ESC>("esc_fixed", 10, false);

  //create publisher to publish steering servo message status with buffer size 10, and latch set to false
  ros::Publisher steering_servo_pub = node_public.advertise<avc_msgs::SteeringServo>("steering_servo_fixed", 10, false);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("control", 1000, controlCallback);

  //create subscriber to subscribe to ESC message topic with queue size set to 1000
  ros::Subscriber esc_sub = node_public.subscribe("/hardware/esc", 1000, escCallback);

  //create subscriber to subscribe to proximity message topic with queue size set to 1000
  ros::Subscriber range_sub = node_public.subscribe("/sensor/proximity", 1000, rangeCallback);

  //create subscriber to subscribe to steering servo message topic with queue size set to 1000
  ros::Subscriber steering_servo_sub = node_public.subscribe("/hardware/steering_servo", 1000, steeringServoCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if robot is within warning distance threshold then reduce throttle proportionally to distance from nearest object
    //robot will stop at danger distance threshold at which point it must be manually moved away from the object
    if ((range_to_nearest > 0) && (range_to_nearest < warning_threshold))
      throttle_percent = throttle_percent * ((range_to_nearest - danger_threshold) / (warning_threshold - danger_threshold));

    //set time of ESC message
    esc_msg.header.stamp = ros::Time::now();

    //set esc message throttle value to latest value
    esc_msg.throttle_percent = throttle_percent;

    //publish esc message
    esc_pub.publish(esc_msg);

    //set time of steering servo message
    steering_servo_msg.header.stamp = ros::Time::now();

    //set steering servo message steering angle value to latest value
    steering_servo_msg.steering_angle = steering_angle;

    //publish steering servo
    steering_servo_pub.publish(steering_servo_msg);

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }

  return 0;
}
