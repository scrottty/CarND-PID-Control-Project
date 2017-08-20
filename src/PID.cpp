#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  
  // Only compute the output if the time isn't too quick
  if (timeDiff > 0.1)
  {
    p_error = cte;
    i_error += cte*timeDiff;
    d_error = (cte - previousError)/timeDiff;
  }
}

void PID::CalcOutput()
{
  currentOutput = Kp*p_error + Ki*i_error + Kd*d_error;
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

