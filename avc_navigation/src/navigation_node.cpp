//navigation node
//this node controls autonomous navigation
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <sstream>
#include <string>
#include <errno.h>
#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <avc_msgs/ChangeControlMode.h>
#include <avc_msgs/Control.h>
#include <avc_msgs/ESC.h>
#include <avc_msgs/Heading.h>
#include <avc_msgs/SteeringServo.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <wiringPi.h>

//math constants
const double EARTH_RADIUS = 6371008.7714;
const double PI = 3.1415926535897;

//global variables
bool autonomous_control = false;
bool autonomous_running = false;
bool mode_change_requested = false;

//global controller variables
std::vector<int> controller_buttons(13, 0);

//global GPS and heading variables
double heading = 0; //[deg]
std::vector<double> gpsFix(2, 0);
std::vector< std::vector<double> > gpsWaypoints;

//pin variables
//must be global so that they can be accessed by callback function
int indicator_LED;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //set all pins LOW
  digitalWrite(indicator_LED, LOW);

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const avc_msgs::Control::ConstPtr& msg)
{

  //change local control mode to match message
  autonomous_control = msg->autonomous_control;

  //set mode change requested flag to true
  mode_change_requested = true;

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_buttons = msg->buttons;

  //if autonomous running button on controller is pressed then toggle autonomous running status
  if ((controller_buttons[1] == 1) && (autonomous_control))
  {

    //set autonomous running status to opposite of current status
    autonomous_running = !autonomous_running;

    //turn on indicator LED during autonomous running
    if (autonomous_running)
      digitalWrite(indicator_LED, HIGH);
    else
      digitalWrite(indicator_LED, LOW);

    //notify that autonomous running is being enabled/disabled
    if (autonomous_running)
      ROS_INFO("[navigation_node] enabling autonomous running");
    else
      ROS_INFO("[navigation_node] disabling autonomous running");

    //reset controller button if pressed to prevent status from toggling twice on one button press
    controller_buttons[1] = 0;

  }

}

//callback function called to process service requests on the disable navigation mode topic
bool disableNavigationCallback(avc_msgs::ChangeControlMode::Request& req, avc_msgs::ChangeControlMode::Response& res)
{

  //if node isn't currently busy then ready to change modes, otherwise not ready to change
  res.ready_to_change = !autonomous_running;

  //output ROS INFO message to inform of mode change request and reply status
  if (req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[navigation_node] mode change requested; changing control modes");
  else if (!req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[navigation_node] ready to change modes status requested; indicating ready to change");
  else if (req.mode_change_requested && !res.ready_to_change)
    ROS_INFO("[navigation_node] mode change requested; indicating node is busy");
  else
    ROS_INFO("[navigation_node] ready to change modes status requested; indicating node is busy");

  //return true to indicate service processing is complete
  return true;

}

//callback function called to process messages on heading topic
void headingCallback(const avc_msgs::Heading::ConstPtr& msg)
{

  //set local values to match message values [deg]
  heading = msg->heading_angle;

}

//TEMPORARY UNTIL ODOMETRY NODE IS FINISHED
//callback function called to process messages on odometry topic
void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  //set local values to match new message values
  gpsFix[0] = msg->latitude / 180 * PI;
  gpsFix[1] = msg->longitude / 180 * PI;

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

  //retrieve indicator LED pin from parameter server
  if (!node_private.getParam("/led/indicator_pin", indicator_LED))
  {
    ROS_ERROR("[navigation_node] indicator LED pin not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve throttle decay constant value from parameter server
  float k_throttle_decay;
  if (!node_private.getParam("/driving/k_throttle_decay", k_throttle_decay))
  {
    ROS_ERROR("[navigation_node] minimum throttle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve maximum throttle value from parameter server [%]
  float maximum_throttle;
  if (!node_private.getParam("/driving/maximum_throttle", maximum_throttle))
  {
    ROS_ERROR("[navigation_node] minimum throttle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve minimum throttle value from parameter server [%]
  float minimum_throttle;
  if (!node_private.getParam("/driving/minimum_throttle", minimum_throttle))
  {
    ROS_ERROR("[navigation_node] minimum throttle not defined in config file: avc_bringup/config/global.yaml");
    ROS_BREAK();
  }

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

  //create publisher to publish ESC message status with buffer size 1, and latch set to false
  ros::Publisher esc_pub = node_public.advertise<avc_msgs::ESC>("esc_raw", 1, false);

  //create publisher to publish steering servo message status with buffer size 1, and latch set to false
  ros::Publisher steering_servo_pub = node_public.advertise<avc_msgs::SteeringServo>("steering_servo_raw", 1, false);

  //create service to process service requests on the disable manual control topic
  ros::ServiceServer disable_navigation_srv = node_public.advertiseService("/control/disable_navigation", disableNavigationCallback);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("/control/control", 1000, controlCallback);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("/control/joy", 1000, controllerCallback);

  //create subscriber to subscribe to heading messages topic with queue size set to 1
  ros::Subscriber heading_sub = node_public.subscribe("/sensor/heading", 1, headingCallback);

  //create subscriber to subscribe to GPS location messages topic with queue size set to 1
  ros::Subscriber nav_sat_fix_sub = node_public.subscribe("/sensor/fix", 1, navSatFixCallback);

  //create subscriber to subscribe to odometry messages topic with queue size set to 1000
  ros::Subscriber odometry_sub = node_public.subscribe("odometry", 1000, odometryCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(indicator_LED, OUTPUT);

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

        //clear GPS waypoint list of previously loaded waypoints
        gpsWaypoints.clear();

        //create local variables to store file line data
        std::string line, latitude, longitude;

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

            //convert latitude and longitude strings into doubles in units of radians [rad]
            std::vector<double> waypoint(2);
            waypoint[0] = std::stod(latitude) / 180 * PI;
            waypoint[1] = std::stod(longitude) / 180 * PI;

            //add waypoint from current line to GPS waypoints list
            gpsWaypoints.push_back(waypoint);

            //output waypoint read from file
            ROS_INFO("[navigation_node] waypoint read from file (latitude, longitude): %lf, %lf", waypoint[0], waypoint[1]);
          }

        }

        //output result of file read
        if (!gpsWaypoints.empty())
          ROS_INFO("[navigation_node] GPS waypoint list read from file, %d total waypoints", int(gpsWaypoints.size()));
        else
          ROS_INFO("[navigation_node] GPS waypoint list read from file but no waypoints found; switch to mapping mode to record waypoints");

      }

    }

    //autonomous (navigation) mode handling
    if (autonomous_control && autonomous_running)
    {

      //if there are a non-zero number of GPS waypoints remaining then run navigation algorithm
      if (!gpsWaypoints.empty())
      {

        //------------------------HEADING TO NEXT WAYPOINT CALCULATION--------------------------

        //calculate delta of latitude from current position to next waypoint in radians
        double delta_longitude = (gpsWaypoints[0][1] - gpsFix[1]);

        //calculate target heading angle from vector pointing from current position to next target waypoint
        float bearing_y = cos(gpsWaypoints[0][0]) * sin(delta_longitude);
        float bearing_x = (cos(gpsFix[0]) * sin(gpsWaypoints[0][0])) - (sin(gpsFix[0]) * cos(gpsWaypoints[0][0]) * cos(delta_longitude));
        float target_heading = atan2(bearing_y, bearing_x);

        //normalize target heading to compass bearing in degrees (0 - 360 deg)
        target_heading = fmod((target_heading / PI * 180) + 360, 360);

        //calculate error between current heading and target heading; positive error values indicate CCW rotation needed
        float error = target_heading - heading;

        //correct error so robot turns the smallest angle possible to reach target heading
        if (error > 180)
          error -= 360;
        else if (error < -180)
          error += 360;

        //output target heading, current heading, and error for debugging purposes
        ROS_INFO("[navigation_node] target heading: %lf, current heading: %lf , error: %f", target_heading, heading, error);

        //output debug data to log
        ROS_DEBUG_NAMED("nav_data", "target heading: %lf, current heading: %lf , error: %f", target_heading, heading, error);

        //set desired servo angle to error value if valid
        if (error > servo_max_angle)
          steering_servo_msg.steering_angle = servo_max_angle;
        else if (error < -servo_max_angle)
          steering_servo_msg.steering_angle = -servo_max_angle;
        else
          steering_servo_msg.steering_angle = error;

        //set time of steering servo message and publish
        steering_servo_msg.header.stamp = ros::Time::now();
        steering_servo_pub.publish(steering_servo_msg);

        //calculate throttle percent from resulting steering angle
        //esc_msg.throttle_percent = maximum_throttle * exp(steering_servo_msg.steering_angle / servo_max_angle * k_throttle_decay);
        esc_msg.throttle_percent = maximum_throttle;

        //if throttle percent is requested below minimum value, set to minimum value
        if (esc_msg.throttle_percent < minimum_throttle)
          esc_msg.throttle_percent = minimum_throttle;

        //set time of ESC message and publish
        esc_msg.header.stamp = ros::Time::now();
        esc_pub.publish(esc_msg);

        //------------------------DISTANCE TO NEXT WAYPOINT CALCULATION-------------------------

        //calculate x and y values of vector from current position to next target waypoint
        double target_delta_x = gpsWaypoints[0][1] - gpsFix[1]; //longitude
        double target_delta_y = gpsWaypoints[0][0] - gpsFix[0]; //latitude

        //convert target delta_x and y values to values in meters (s = r * theta)
        target_delta_x = target_delta_x  * EARTH_RADIUS;
        target_delta_y = target_delta_y  * EARTH_RADIUS;

        //check if robot is within defined distance of waypoint, notify and set target to next waypoint if true
        if (sqrt(pow(target_delta_x, 2) + pow(target_delta_y, 2)) < waypoint_radius)
        {

          //notify that waypoint has been reached
          ROS_INFO("[navigation_node] target reached; navigating to next waypoint (%d remaining)", int(gpsWaypoints.size()));

          //remove current waypoint from list
          gpsWaypoints.erase(gpsWaypoints.begin());

        }

      }
      //end autonomous running and notify if there are no waypoints remaining in list
      else
      {

        //end autonomous running
        autonomous_running = false;

        //flash LED twice to indicate goal has been reached
        for (int i = 0; i < 2; i++)
        {

          //flash LED
          digitalWrite(indicator_LED, LOW);
          delay(500);
          digitalWrite(indicator_LED, HIGH);

          //delay between flashes or turn off LED if flashing is finished
          if (i < 1)
            delay(500);
          else
            digitalWrite(indicator_LED, LOW);

        }

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
