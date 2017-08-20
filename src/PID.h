#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double previousError;
  
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  double timeDiff;
  
  double currentOutput;
  
  

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  // Calc output
  void CalcOutput();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
