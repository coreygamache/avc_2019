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
float range_front = 9999; //front sensor range to nearest object [m]
float range_left = 9999; //left sensor range to nearest object [m]
float range_right = 9999; //right sensor range to nearest object [m]
float throttle_percent = 0; //requested throttle position [%]
float steering_angle = 0; //requested steering angle [deg]


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

//callback function called to process messages on front range topic
void rangeFrontCallback(const sensor_msgs::Range::ConstPtr& msg)
{

  //change local control mode to match message
  range_to_nearest = msg->range;

}

//callback function called to process messages on left range topic
void rangeLeftCallback(const sensor_msgs::Range::ConstPtr& msg)
{

  //change local control mode to match message
  range_to_nearest = msg->range;

}

//callback function called to process messages on right range topic
void rangeRightCallback(const sensor_msgs::Range::ConstPtr& msg)
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

  //retrieve collision avoidance braking constant value from parameter server
  float k_collision_brake;
  if (!node_private.getParam("/driving/k_collision_brake", k_collision_brake))
  {
    ROS_ERROR("[collision_avoidance_node] collision avoidance braking constant not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve collision avoidance steering constant value from parameter server
  float k_collision_steer;
  if (!node_private.getParam("/driving/k_collision_steer", k_collision_steer))
  {
    ROS_ERROR("[collision_avoidance_node] collision avoidance steering constant not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve minimum throttle value from parameter server [%]
  float minimum_throttle;
  if (!node_private.getParam("/driving/minimum_throttle", minimum_throttle))
  {
    ROS_ERROR("[collision_avoidance_node] minimum throttle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/collision_avoidance_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[collision_avoidance_node] collision avoidance node refresh rate not defined in config file: avc_control/config/control.yaml");
    ROS_BREAK();
  }

  //retrieve maximum steering angle value from parameter server [deg]
  loat max_steering_angle;
  if (!node_private.getParam("/steering_servo/max_rotation_angle", max_steering_angle))
  {
    ROS_ERROR("[collision_avoidance_node] maximum steering angle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }
  f

  //retrieve algorithm engagement threshold distance from parameter server
  float threshold_distance;
  if (!node_private.getParam("/control/collision_avoidance_node/threshold_distance", threshold_distance))
  {
    ROS_ERROR("[collision_avoidance_node] threshold distance not defined in config file: avc_control/config/control.yaml");
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
  ros::Subscriber esc_sub = node_public.subscribe("/navigation/esc_raw", 1000, escCallback);

  //create subscriber to subscribe to front proximity message topic with queue size set to 1000
  ros::Subscriber range_sub = node_public.subscribe("/sensor/proximity/front", 1000, rangeFrontCallback);

  //create subscriber to subscribe to left proximity message topic with queue size set to 1000
  ros::Subscriber range_sub = node_public.subscribe("/sensor/proximity/left", 1000, rangeLeftCallback);

  //create subscriber to subscribe to right proximity message topic with queue size set to 1000
  ros::Subscriber range_sub = node_public.subscribe("/sensor/proximity/right", 1000, rangeRightCallback);

  //create subscriber to subscribe to steering servo message topic with queue size set to 1000
  ros::Subscriber steering_servo_sub = node_public.subscribe("/navigation/steering_servo_raw", 1000, steeringServoCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //only enable collisioon avoidance logic in autonomous control mode
    if (autonomous_control)
    {

      //engage collision avoidance algorithm if there's an obstacle within the threshold distance
      if (range_front < threshold_distance)
      {

        //calculate steering correction value
        float steering_correction = k_collision_steer * (1 - (range_front / threshold_distance));

        //handle case: steering left is safer
        if (range_left < range_right)
          steering_angle += steering_correction;
        //handle case: steering right is safer
        else if (range_left > range_right)
          steering_angle -= steering_correction;
        //handle case: left and right are equally safe
        else
        {

          //continue steering in current steering direction (default left if current steering angle is zero)
          if (steering_angle >= 0)
            steering_angle += steering_correction;
          else
            steering_angle -= steering_correction;

        }

      }

      //validate steering angle
      if (steering_angle < -max_steering_angle)
        steering_angle = -max_steering_angle;
      else if (steering_angle > max_steering_angle)
        steering_angle = max_steering_angle;

      //set time of steering servo message
      steering_servo_msg.header.stamp = ros::Time::now();

      //set steering servo message steering angle value to latest value
      steering_servo_msg.steering_angle = steering_angle;

      //publish steering servo
      steering_servo_pub.publish(steering_servo_msg);

      throttle_percent = throttle_percent * k_collision_brake * (1 - (range_front / threshold_distance));

      //if throttle percent is below set threshold then set to zero
      if (throttle_percent < minimum_throttle)
        throttle_percent = 0;

      //set time of ESC message
      esc_msg.header.stamp = ros::Time::now();

      //set esc message throttle value to latest value
      esc_msg.throttle_percent = throttle_percent;

      //publish esc message
      esc_pub.publish(esc_msg);



    }

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }

  return 0;
}
