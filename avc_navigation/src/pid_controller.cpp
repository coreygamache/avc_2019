//include header
#include <pid_controller.hpp>


//default constructor
PIDController::PIDController(double kp, double ki, double kd, double min, double max, double dt)
{

  //set class variable values to passed values
  this->_kp = kp;
  this->_ki = ki;
  this->_kd = kd;
  this->_min_output = min;
  this->_max_output = max;
  this->_dt = dt;

  //initialize other variables
  this->_integral_value = 0;
  this->_previous_error = 0;

  //set initial set point to 0
  this->_set_point = 0;

}

//default destructor
PIDController::~PIDController() {}

//set functions

//set class set_point value
void PIDController::setSetPoint(double set_point){
  this->_set_point = set_point;
}

//other functions

//calculate output based on input with predefined set point
double PIDController::calculate(double input)
{

  //initialize output value
  double output = 0;

  //calculate current error
  double error = this->_set_point - input;

  //add proportional term to output
  output += this->_kp * error;

  //update integral value and add integral term to output
  this->_integral_value += error * this->_dt;
  output += this->_ki * this->_integral_value;

  //add derivative term to output
  output += this->_kd * ((error - this->_previous_error) / this->_dt);

  //verify output is within specified range
  if (output > this->_max_output)
    output = this->_max_output;
  else if (output < this->_min_output)
    output = this->_min_output;

  //set previous error value to current error
  this->_previous_error = error;

  //return calculated output value
  return output;

}

//calculate output based on input and new set point
double PIDController::calculate(double set_point, double input)
{

  //set set_point to passed value
  this->setSetPoint(set_point);

  //call calculate function with passed input and return result
  return this->calculate(input);

}
