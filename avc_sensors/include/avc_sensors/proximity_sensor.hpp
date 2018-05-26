#ifndef PROXIMITY_SENSOR_HPP
#define PROXIMITY_SENSOR_HPP

class ProximitySensor
{
  public:
    ProximitySensor(int echo, int trigger);
    double getDistance(int timeout);

    //get functions
    int getEchoPin();
    int getTriggerPin();

    //set functions
    void setEchoPin(int echo);
    void setTriggerPin(int trigger);

  private:
    int echoPin;
    int triggerPin;

};

#endif
