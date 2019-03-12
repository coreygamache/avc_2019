//IMU node
//outputs IMU data
#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <avc_msgs/Heading.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <signal.h>

//external library includes
#include <RTIMULib.h>

//gravity to m/s^2 conversion factor
static const double G_TO_MPSS = 9.80665;

//math constants
const double PI = 3.1415926535897;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting imu_node");

  //initialize node and create node handler
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve calibration file path from parameter server [RTIMULib parameter]
  std::string calibration_file_path;
  if (!node_private.getParam("/sensor/imu/calibration_file_path", calibration_file_path))
  {
    ROS_ERROR("[imu_node] calibration file not found");
    ROS_BREAK();
  }
  //ROS_INFO("[imu_node] calibration file path: %s", calibration_file_path.c_str()); //output calibration file path (for testing)

  //retrieve calibration file name from parameter server [RTIMULib parameter]
  std::string calibration_file_name = "RTIMULib";
  if (!node_private.getParam("/sensor/imu/calibration_file_name", calibration_file_name))
  {
    ROS_WARN_STREAM("[imu_node] no calibration file name provided, using default: " << calibration_file_name);
  }

  //get frame id parameter [RTIMULib parameter]
  std::string frame_id = "imu_link";
  if (!node_private.getParam("/sensor/imu/frame_id", frame_id))
  {
    ROS_WARN_STREAM("[imu_node] no frame_id provided, using default: " << frame_id);
  }

  //get refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/sensor/imu/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[imu_node] sensor refresh rate not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //create RTIMUSettings type object called imu_settings to set initial IMU settings that will later be used to create IMU object
  RTIMUSettings *imu_settings = new RTIMUSettings(calibration_file_path.c_str(), calibration_file_name.c_str());

  //create RTIMU type object called imu using previously determined settings
  RTIMU *imu = RTIMU::createIMU(imu_settings);

  //make sure IMU was detected
  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
  {
    ROS_ERROR("[imu_node] IMU not found");
    ROS_BREAK();
  }

  //initialize IMU
  if (!imu->IMUInit())
  {
    ROS_ERROR("[imu_node] failed to initialize IMU");
    ROS_BREAK();
  }

  //retrieve slerp power from parameter server
  float slerp_power;
  if (!node_private.getParam("/sensor/imu/slerp_power", slerp_power))
  {
    ROS_ERROR("[imu_node] IMU slerp power not defined in config file: avc_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //set fusion coefficient and enable gyro, accelerometer, and compass
  imu->setSlerpPower(slerp_power);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);

  //create geometry_msgs/Imu type message to publish IMU data
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = frame_id;

  //retrieve IMU orientation covariance values from parameter server
  std::vector<double> orientation_covariance;
  if (!node_private.getParam("orientation_covariance", orientation_covariance) || orientation_covariance.size() != 9)
  {
    orientation_covariance.assign(9, 0);
  }
  std::copy(orientation_covariance.begin(), orientation_covariance.end(), std::begin(imu_msg.orientation_covariance));

  //retrieve IMU angular velocity covariance values from parameter server
  std::vector<double> angular_velocity_covariance;
  if (!node_private.getParam("angular_velocity_covariance", angular_velocity_covariance) || angular_velocity_covariance.size() != 9)
  {
    angular_velocity_covariance.assign(9, 0);
  }
  std::copy(angular_velocity_covariance.begin(), angular_velocity_covariance.end(), std::begin(imu_msg.angular_velocity_covariance));

  //retrieve IMU linear acceleration covariance values from parameter server
  std::vector<double> linear_acceleration_covariance;
  if (!node_private.getParam("linear_acceleration_covariance", linear_acceleration_covariance) || linear_acceleration_covariance.size() != 9)
  {
    linear_acceleration_covariance.assign(9, 0);
  }
  std::copy(linear_acceleration_covariance.begin(), linear_acceleration_covariance.end(), std::begin(imu_msg.linear_acceleration_covariance));

  //create disable mapping service object and set default parameters
  avc_msgs::Heading heading_msg;
  heading_msg.header.frame_id = "0";

  //create sensor_msgs/MagneticField type message to publish compass data
  sensor_msgs::MagneticField compass_msg;
  compass_msg.header.frame_id = "0";

  //set compass message magnetic_field_covariance values to zero to indicate unknown
  std::vector<double> magnetic_field_covariance(9, 0);
  std::copy(magnetic_field_covariance.begin(), magnetic_field_covariance.end(), std::begin(compass_msg.magnetic_field_covariance));

  //create publisher to publish compass messages with buffer size 1, and latch set to false
  ros::Publisher compass_pub = node_public.advertise<sensor_msgs::MagneticField>("compass", 1, false);

  //create publisher to publish compass heading messages with buffer size 1, and latch set to false
  ros::Publisher heading_pub = node_public.advertise<avc_msgs::Heading>("heading", 1, false);

  //create publisher to publish IMU messages with buffer size 1, and latch set to false
  ros::Publisher imu_pub = node_public.advertise<sensor_msgs::Imu>("imu", 1, false);

  //set refresh rate of ROS loop to defined refresh rate of sensor parameter
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    if (imu->IMURead())
    {

      //get current IMU data
      RTIMU_DATA imu_data = imu->getIMUData();

      //set IMU message headers
      imu_msg.header.stamp = ros::Time::now();

      //set IMU message angles
      imu_msg.orientation.x = imu_data.fusionQPose.x();
      imu_msg.orientation.y = imu_data.fusionQPose.y();
      imu_msg.orientation.z = imu_data.fusionQPose.z();
      imu_msg.orientation.w = imu_data.fusionQPose.scalar();

      //set IMU message angular velocities
      imu_msg.angular_velocity.x = imu_data.gyro.x();
      imu_msg.angular_velocity.y = imu_data.gyro.y();
      imu_msg.angular_velocity.z = imu_data.gyro.z();
      //ROS_INFO("velocity (x, y, z): %f, %f, %f", imu_data.gyro.x(), imu_data.gyro.y(), imu_data.gyro.z());

      //set IMU message linear accelerations
      imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
      imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
      imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

      //publish IMU message
      imu_pub.publish(imu_msg);

      //if compass reading is value then get data and publish compass message
      if (imu_data.compassValid)
      {

        //set compass message headers
        compass_msg.header.stamp = ros::Time::now();

        //set compass message magnetic field values
        //values are returned from IMU in units of microTeslas, therefore must be divided by 10^6 to convert to Teslas
        compass_msg.magnetic_field.x = imu_data.compass.x() / 1000000;
        compass_msg.magnetic_field.y = imu_data.compass.y() / 1000000;
        compass_msg.magnetic_field.z = imu_data.compass.z() / 1000000;

        //publish compass message
        compass_pub.publish(compass_msg);

        //set heading message headers
        heading_msg.header.stamp = ros::Time::now();

        //convert from yaw to heading
        //yaw ranges from 0 to +/- 180 degrees as measured counter-clockwise from compass east
        //heading ranges from 0 to 360 degrees as measured clockwise from compass north
        //heading = ()(-yaw + 90) + 360) % 360

        //set heading angle of heading msg to negative yaw value from IMU to change positive rotation direction to match bearing coordinate frame [rad]
        heading_msg.heading_angle = -1 * imu_data.fusionPose.z();

        //convert from yaw [rad] to magnetic heading [deg]
        heading_msg.heading_angle = (heading_msg.heading_angle / PI * 180) + 90;

        //normalize yaw value to compass heading in degrees (0 - 360 deg)
        heading_msg.heading_angle = fmod(heading_msg.heading_angle + 360, 360);

        //publish heading message
        heading_pub.publish(heading_msg);

        //output debug data to log
        ROS_DEBUG_NAMED("heading_data", "current yaw: %f", imu_data.fusionPose.z());
        ROS_DEBUG_NAMED("heading_data", "current heading: %f", heading_msg.heading_angle);

      }

    }

    //process callback functions
    ros::spinOnce();

    //sleep until next IMU reading
    ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();

    //sleep until next sensor reading
    //loop_rate.sleep();

  }
  return 0;
}
