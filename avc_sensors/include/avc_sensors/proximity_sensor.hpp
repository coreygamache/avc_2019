#ifndef DEF_PROXIMITY_SENSOR
#define DEF_PROXIMITY_SENSOR

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
