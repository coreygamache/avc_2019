//navigation node
//this node controls autonomous navigation
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <sstream>
#include <string>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <avc_msgs/ChangeControlMode.h>
#include <avc_msgs/Control.h>
#include <avc_msgs/ESC.h>
#include <avc_msgs/SteeringServo.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h> //TEMPORARY UNTIL ODOMETRY NODE IS FINISHED
#include <signal.h>
#include <wiringPi.h>

//global variables
bool autonomous_control = false;
bool autonomous_running = false;
bool mode_change_requested = false;

//global controller variables
std::vector<int> controller_buttons(13, 0);

//global GPS and heading variables
float heading; //[deg]
std::vector<double> gpsFix(2, 0); //TEMPORARY UNTIL ODOMETRY NODE IS FINISHED
std::vector< std::vector<double> > gpsWaypoints;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on compass topic
void compassCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{

  //set local heading value to heading derived from compass data
  heading = atan2(msg->magnetic_field.y, msg->magnetic_field.x) * 180 / 3.14159;

}

//callback function called to process messages on control topic
void controlCallback(const avc_msgs::Control::ConstPtr& msg)
{

  //verify that local mode matches global mode
  if (autonomous_control != msg->autonomous_control)
  {

    //modes do not match; send notification and shut down node
    ROS_INFO("[navigation_node] local control mode does not match global control mode; killing program");
    ROS_BREAK();

  }

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_buttons = msg->buttons;

  //if autonomous running button on controller is pressed then toggle autonomous running status
  if (controller_buttons[1] == 1)
  {

    //set autonomous running status to opposite of current status
    autonomous_running = !autonomous_running;

    //reset controller button if pressed to prevent status from toggling twice on one button press
    controller_buttons[7] = 0;

  }

}

//callback function called to process service requests on the disable navigation mode topic
bool disableNavigationCallback(avc_msgs::ChangeControlMode::Request& req, avc_msgs::ChangeControlMode::Response& res)
{

  //if node isn't currently busy then ready to change modes, otherwise not ready to change
  res.ready_to_change = true;

  //output ROS INFO message to inform of mode change request and reply status
  if (req.mode_change_requested && res.ready_to_change)
  {

    //change modes
    autonomous_control = !autonomous_control;
    mode_change_requested = true;

    //output notification
    ROS_INFO("[navigation_node] mode change requested; changing control modes");

  }
  else if (!req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[navigation_node] ready to change modes status requested; indicating ready to change");
  else if (req.mode_change_requested && !res.ready_to_change)
    ROS_INFO("[navigation_node] mode change requested; indicating node is busy");
  else
    ROS_INFO("[navigation_node] ready to change modes status requested; indicating node is busy");

  //return true to indicate service processing is complete
  return true;

}

//TEMPORARY UNTIL ODOMETRY NODE IS FINISHED
//callback function called to process messages on odometry topic
void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  //set local values to match message values
  gpsFix[0] = msg->latitude;
  gpsFix[1] = msg->longitude;

}

//callback function called to process messages on odometry topic
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  //set local values to match message values
  //odometry

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting navigation_node");

  //initialize node and create node handler
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve map waypoint delay from parameter server [ms]
  std::string output_file_path;
  if (!node_private.getParam("/mapping/output_file_path", output_file_path))
  {
    ROS_ERROR("[navigation_node] GPS waypoint output file path not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/navigation/navigation_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[navigation_node] navigation node refresh rate not defined in config file: avc_navigation/config/navigation.yaml");
    ROS_BREAK();
  }

  //retrieve steering servo max rotation angle from parameter server
  float servo_max_angle;
  if (!node_private.getParam("/steering_servo/max_rotation_angle", servo_max_angle))
  {
    ROS_ERROR("[navigation_node] steering servo max rotation angle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  float waypoint_radius;
  if (!node_private.getParam("/navigation/navigation_node/waypoint_radius", waypoint_radius))
  {
    ROS_ERROR("[navigation_node] waypoint radius not defined in config file: avc_navigation/config/navigation.yaml");
    ROS_BREAK();
  }

  //create ESC message object and set default parameters
  avc_msgs::ESC esc_msg;
  esc_msg.header.frame_id = "0";

  //create steering servo message object and set default parameters
  avc_msgs::SteeringServo steering_servo_msg;
  steering_servo_msg.header.frame_id = "0";

  //create publisher to publish ESC message status with buffer size 10, and latch set to false
  ros::Publisher esc_pub = node_public.advertise<avc_msgs::ESC>("esc", 10, false);

  //create publisher to publish steering servo message status with buffer size 10, and latch set to false
  ros::Publisher steering_servo_pub = node_public.advertise<avc_msgs::SteeringServo>("steering_servo", 10, false);

  //create service to process service requests on the disable manual control topic
  ros::ServiceServer disable_navigation_srv = node_public.advertiseService("/control/disable_navigation", disableNavigationCallback);

  //create subscriber to subscribe to compass messages topic with queue size set to 1000
  ros::Subscriber compass_sub = node_public.subscribe("/sensor/compass", 1000, compassCallback);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("/control/control", 1000, controlCallback);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("/control/joy", 1000, controllerCallback);

  //TEMPORARY UNTIL ODOMETRY NODE IS FINISHED
  //create subscriber to subscribe to GPS location messages topic with queue size set to 1000
  ros::Subscriber nav_sat_fix_sub = node_public.subscribe("/sensor/fix", 1000, navSatFixCallback);

  //create subscriber to subscribe to odometry messages topic with queue size set to 1000
  ros::Subscriber odometry_sub = node_public.subscribe("odometry", 1000, odometryCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    if (mode_change_requested)
    {

      //set mode change requested to false to prevent two mode changes on change request
      mode_change_requested = false;

      //if autonomous control is enabled then load most recent GPS waypoint list from file
      if (autonomous_control)
      {

        //create local variables and vector to store file line data and GPS waypoints
        std::string line, latitude, longitude;
        std::vector< std::vector<double> > gpsWaypoints;

        //open input file to read GPS waypoints
        std::fstream input_file(output_file_path.c_str(), std::fstream::in);

        //if file was read successfully then skip header line
        if (input_file && input_file.good())
          std::getline(input_file, latitude);

        //read waypoints into list from file line by line
        while (input_file.good())
        {

          //read next line in file
          std::getline(input_file, line);

          //convert string read from file to istringstream for parsing by comma
          std::istringstream line_stream(line);

          //get latitude and longitude from current line in file
          std::getline(line_stream, latitude, ',');
          std::getline(line_stream, longitude, '\n');

          //store waypoint in list of waypoints if latitude and longitude values are valid
          if (!latitude.empty() && !longitude.empty())
          {

            //convert latitude and longitude strings into doubles
            std::vector<double> waypoint(2);
            waypoint[0] = std::stod(latitude);
            waypoint[1] = std::stod(longitude);

            //add waypoint from current line to GPS waypoints list
            gpsWaypoints.push_back(waypoint);

          }

        }

        //output result of file read
        if (gpsWaypoints.size() > 0)
          ROS_INFO("[navigation_node] GPS waypoint list read from file, %d total waypoints", int(gpsWaypoints.size()));
        else
          ROS_INFO("[navigation_node] GPS waypoint list read from file but no waypoints found; switch to mapping mode to record waypoints");
      }

    }

    //autonomous (navigation) mode handling
    if (autonomous_control && autonomous_running)
    {

      //if there are a non-zero number of GPS waypoints remaining then run navigation algorithm
      if (gpsWaypoints.size() > 0)
      {

        //calculate x and y values of vector from current position to next target waypoint
        double delta_x = gpsWaypoints[0][1] - gpsFix[1];
        double delta_y = gpsWaypoints[0][0] - gpsFix[0];

        //calculate target heading angle from vector pointing from current position to next target waypoint
        float target_heading = atan2(delta_y, delta_x) * 180 / 3.14159;

        //calculate error between target heading and current heading
        float error = target_heading - heading;

        //set desired servo angle to error value if valid
        if (error > servo_max_angle)
          steering_servo_msg.steering_angle = servo_max_angle;
        else if (error < -servo_max_angle)
          steering_servo_msg.steering_angle = -servo_max_angle;
        else
          steering_servo_msg.steering_angle = error;

        //calculate throttle percent from resulting steering angle
        esc_msg.throttle_percent = (fabs(error) / servo_max_angle) * 100;

        //set time of ESC message and publish
        esc_msg.header.stamp = ros::Time::now();
        esc_pub.publish(esc_msg);

        //set time of steering servo message and publish
        steering_servo_msg.header.stamp = ros::Time::now();
        steering_servo_pub.publish(steering_servo_msg);

        //convert delta_x and y values to values in meters (s = r * theta)
        delta_x = 6378137 * ((delta_x / 180) *  3.14159);
        delta_y = 6378137 * ((delta_y / 180) *  3.14159);

        //check if robot is within defined distance of waypoint, and set target to next waypoint if true
        if (sqrt(pow(delta_x, 2) + pow(delta_y, 2)) < waypoint_radius)
          gpsWaypoints.erase(gpsWaypoints.begin());

      }
      else
      {

        //inform that there are no remaining GPS waypoints to navigate to
        ROS_INFO("[navigation_node] no GPS waypoints remaining in list; navigation complete");
        ROS_INFO("[navigation_node] switch to mapping mode and back to reload GPS waypoints");

      }

    }
    //if autonomous running is disabled then stop robot and reset steering angle
    else if (autonomous_control && !autonomous_running)
    {

      //reset ESC msg, set time, and publish
      esc_msg.header.stamp = ros::Time::now();
      esc_msg.throttle_percent = 0;
      esc_pub.publish(esc_msg);

      //reset steering msg, set time, and publish
      steering_servo_msg.header.stamp = ros::Time::now();
      steering_servo_msg.steering_angle = 0;
      steering_servo_pub.publish(steering_servo_msg);

    }

    //process callback functions
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
