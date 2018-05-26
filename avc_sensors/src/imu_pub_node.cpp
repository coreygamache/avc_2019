//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

//external library includes
#include <RTIMULib.h>

//gravity to m/s^2 conversion factor
static const double G_TO_MPSS = 9.80665;

int main(int argc, char **argv)
{

  //initialize node
  ros::init(argc, argv, "imu_pub_node");

  //create private NodeHandle for getting IMU parameters and publishing
  ros::NodeHandle node_private("~");

  //get calibration file path parameter
  std::string calibration_file_path;
  if (!node_private.getParam("calibration_file_path", calibration_file_path))
  {
    ROS_ERROR("calibration file not found");
    ROS_BREAK();
  }

  //get calibration file name parameter
  std::string calibration_file_name = "RTIMULib";
  if (!node_private.getParam("calibration_file_name", calibration_file_name))
  {
    ROS_WARN_STREAM("no calibration file name provided, using default: " << calibration_file_name);
  }

  //get frame id parameter
  std::string frame_id = "imu_link";
  if (!node_private.getParam("frame_id", frame_id))
  {
    ROS_WARN_STREAM("no frame_id provided, using default: " << frame_id);
  }

  //create RTIMUSettings type object called imu_settings to set initial IMU settings that will later be used to create IMU object
  RTIMUSettings *imu_settings = new RTIMUSettings(calibration_file_path.c_str(), calibration_file_name.c_str());

  //create RTIMU type object called imu using previously determined settings
  RTIMU *imu = RTIMU::createIMU(imu_settings);

  //make sure IMU was detected
  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
  {
    ROS_ERROR("IMU not found");
    ROS_BREAK();
  }

  //initialize IMU
  if (!imu->IMUInit())
  {
    ROS_ERROR("failed to initialize IMU");
    ROS_BREAK();
  }

  //set fusion coefficient and enable gyro, accelerometer, and compass
  imu->setSlerpPower(0.02);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);

  //create geometry_msgs/Imu type message to publish IMU data
  sensor_msgs::Imu imu_msg;

  //get IMU orientation covariance values
  std::vector<double> orientation_covariance;
  if (node_private.getParam("orientation_covariance", orientation_covariance) && orientation_covariance.size() == 9)
  {
    for (int i=0; i < 9; i++)
    {
      imu_msg.orientation_covariance[i] = orientation_covariance[i];
    }
  }
  else
  {
    imu_msg.orientation_covariance[0] = -1;
  }

  //get IMU angular velocity covariance values
  std::vector<double> angular_velocity_covariance;
  if (node_private.getParam("angular_velocity_covariance", angular_velocity_covariance) && angular_velocity_covariance.size() == 9)
  {
    for (int i=0; i < 9; i++)
    {
      imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance[i];
    }
  }
  else
  {
    imu_msg.angular_velocity_covariance[0] = -1;
  }

  //get IMU linear acceleration covariance values
  std::vector<double> linear_acceleration_covariance;
  if (node_private.getParam("linear_acceleration_covariance", linear_acceleration_covariance) && linear_acceleration_covariance.size() == 9)
  {
    for (int i=0; i < 9; i++)
    {
      imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
    }
  }
  else
  {
    imu_msg.linear_acceleration_covariance[0] = -1;
  }

  //create sensor_msgs/MagneticField type message to publish compass data
  sensor_msgs::MagneticField compass_msg;

  //set compass message magnetic_field_covariance values to zero to indicate unknown
  for (int i=0; i < 9; i++)
  {
    compass_msg.magnetic_field_covariance[i] = 0;
  }

  //create publishers to publish IMU and compass messages with buffer size 10, and latch set to false
  ros::Publisher imu_pub = node_private.advertise<sensor_msgs::Imu>("imu", 10, false);
  ros::Publisher compass_pub = node_private.advertise<sensor_msgs::MagneticField>("compass", 10, false);

  //set refresh rate to 10 hz
  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    if (imu->IMURead())
    {
      RTIMU_DATA imu_data = imu->getIMUData();

      //set IMU message headers
      imu_msg.header.frame_id = frame_id;
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

      //set IMU message linear accelerations
      imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
      imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
      imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

      //publish IMU message
      imu_pub.publish(imu_msg);

      if (imu_data.compassValid)
      {

        //set compass message headers
        compass_msg.header.frame_id = frame_id;
        compass_msg.header.stamp = ros::Time::now();

        //set compass message magnetic field values
        //values are returned from IMU in units of microTeslas, therefore must be divided by 10^6 to convert to Teslas
        compass_msg.magnetic_field.x = imu_data.compass.x() / 1000000;
        compass_msg.magnetic_field.y = imu_data.compass.y() / 1000000;
        compass_msg.magnetic_field.z = imu_data.compass.z() / 1000000;

        //publish compass message
        compass_pub.publish(compass_msg);

      }

    }

    //spin once because ROS
    ros::spinOnce();

    //sleep until next IMU reading
    ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();

  }
  return 0;
}
