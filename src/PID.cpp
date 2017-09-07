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
  
  prevTime = std::chrono::system_clock::now();
  previousError = 0;
}

void PID::UpdateError(double cte) {
  
  currentTime = std::chrono::system_clock::now();
  timeDiff_duration = currentTime - prevTime;
  timeDiff = timeDiff_duration.count();
  
  // Only compute the output if the time isn't too quick
  if (timeDiff >= 0.1)
  {
    prevTime = currentTime;
    
    p_error = cte;
    i_error += cte*timeDiff;
    if (previousError != 0) // signifies first loop
    {
        d_error = (cte - previousError)/timeDiff;
    }
    previousError = cte;
  }
}

void PID::CalcOutput()
{
  currentOutput = Kp*p_error + Ki*i_error + Kd*d_error;
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

