//control node
//this node controls control mode changing via controller button press
#include <errno.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //kill the node if the emergency stop button is pressed
  if (msg->buttons[2] == 1)
    ros::shutdown();

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting emergency_stop_node");

  //initialize node and create node handler
  ros::init(argc, argv, "emergency_stop_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("joy", 1000, controllerCallback);

  //process subscribed messages through callback functions indefinitely
  ros::spin();

  return 0;
}
