//ROS includes
#include <ros/ros.h>

//external includes
#include <wiringPi.h>

//include header
#include <proximity_sensor.hpp>

//default constructor
ProximitySensor::ProximitySensor(int echo, int trigger)
{

  //run wiringPi setup functions
  wiringPiSetup();

  //set pins to given pin numbers
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

  //set pin to provided value, ensuring input is valid
  //default to GPIO pin 23 on invalid input
  if ((echo < 2) || (echo > 27))
  {
    this->echoPin = 23;
  }
  else
  {
    this->echoPin = echo;
  }

  //designate pin as input and disable pull-up resistor
  pinMode(this->echoPin, INPUT);
  digitalWrite(this->echoPin, LOW);

}

void ProximitySensor::setTriggerPin(int trigger)
{

  //set pin to provided value, ensuring input is valid
  //default to GPIO pin 24 on invalid input
  if ((trigger < 2) || (trigger > 27))
  {
    this->triggerPin = 24;
  }
  else
  {
    this->triggerPin = trigger;
  }

  //designate pin as output and disable pull-up resistor
  pinMode(this->triggerPin, OUTPUT);
  digitalWrite(this->triggerPin, LOW);

}

//getDistance       get distance to nearest object in sensor's field of view
//params:
//  int timeout     time to wait for echo in milliseconds (optional, default 50)
//returns:
//  double          distance to nearest object in meters
double ProximitySensor::getDistance(int timeout = 30)
{

  //convert timeout in milliseconds to microseconds
  timeout *= 1000;

  //send sonar pulse
  digitalWrite(this->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->triggerPin, LOW);

  //create and set start time to current time
  long start_time = micros();

  //wait for start of HIGH signal from echo pin
  //return -1 if time waiting exceeds timeout duration to indicate error
  while (digitalRead(this->echoPin) == LOW)
  {
    if ((micros() - start_time) >= timeout)
    {
      return -1;
    }
  }

  //set time to current time (start of HIGH signal)
  start_time = micros();

  //wait for end of HIGH signal from echo pin
  //return -1 if time waiting exceeds timeout duration to indicate error
  while (digitalRead(this->echoPin) == HIGH)
  {
    if ((micros() - start_time) >= timeout)
    {
      return -1;
    }
  }

  //set end time to current time (end of HIGH signal)
  long end_time = micros();

  //return distance to nearest object in meters
  return (end_time - start_time) * 0.00034 / 2;

}
