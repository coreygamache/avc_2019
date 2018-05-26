#include "ros/ros.h"
#include "proximity_sensor.hpp"
#include "wiringPi.h"

//default constructor
ProximitySensor::ProximitySensor(int echo, int trigger)
{
  setEchoPin(echo);
  setTriggerPin(trigger);
}

int ProximitySensor::getEchoPin()
{
  return this->echoPin;
}

int ProximitySensor::getTriggerPin()
{
  return this->triggerPin;
}

void ProximitySensor::setEchoPin(int echo)
{
  pinMode(this->echoPin, INPUT);
  digitalWrite(this->echoPin, LOW);
}

void ProximitySensor::setTriggerPin(int trigger)
{
  pinMode(this->triggerPin, OUTPUT);
  digitalWrite(this->trigger, LOW);
}

//getDistance       get distance to nearest object in sensor's field of view
//params:
//  int timeout     time to wait for echo in milliseconds (optional, default 50)
//returns:
//  double          distance to nearest object in meters
double ProximitySensor::getDistance(int timeout = 20)
{

  //convert timeout in milliseconds to microseconds
  timeout *= 1000;

  //send sonar pulse
  digitalWrite(this->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->triggerPin, LOW);

  long start_time = micros();

  //wait for start of signal from echo pin
  //return -1 if time waiting exceeds timeout duration to indicate error
  while (this->echoPin == LOW)
  {
    if ((micros() - start_time) >= timeout)
    {
      return -1;
    }
  }
  start_time = micros();

  //wait for end of signal from echo pin
  //return -1 if time waiting exceeds timeout duration to indicate error
  while (this->echoPin == LOW)
  {
    if ((micros() - start_time) >= timeout)
    {
      return -1;
    }
  }
  long end_time = micros();

  //return distance to nearest object in meters
  return (end_time - start_time) * 0.00034 / 2;

}
