#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController
{
  public:

    //constructors and destructors
    PIDController(double kp, double ki, double kd, double min, double max, double dt); //create PID controller with default set point = 0
    ~PIDController();

    //set functions
    void setSetPoint(double set_point);

    //other functions
    double calculate(double input); //calculate output based on input with predefined set point
    double calculate(double set_point, double input); //calculate output based on input and new set point

  private:
    double _dt;
    double _kd;
    double _ki;
    double _kp;
    double _integral_value;
    double _max_output;
    double _min_output;
    double _previous_error;
    double _set_point;

};

#endif
