# avc_sensors
The avc_sensors package provides a number of drivers for integrating the various sensors used into the navigation software.<br><br>
__GPS__: The [__nmea_navsat_driver__](http://wiki.ros.org/nmea_navsat_driver) package is used for GPS integration and launched with the appropriate launch file, and thus there is no gps_pub_node.<br><br>
__IMU__: The [__RTIMULib2__](https://github.com/richardstechnotes/RTIMULib2) library is used for integration of the MPU-9250 IMU. The IMU node, imu_pub_node, is merely a wrapper for this library to publish the appropriate information to ROS. Information is published to ROS as sensor_msgs/Imu and sensor_msgs/MagneticField data on the /imu_pub_node/imu and /imu_pub_node/compass topics, respectively.<br><br>
__LiDAR__:<br><br>
__Proximity Sensors__: A custom proximity sensor library, proximity_sensor, is used by proximity_pub_node to interface with the HC-SR04 ultrasonic sensor. Information is published to ROS as sensor_msgs/Range data on the /proximity_pub_node/proximity topic.
