#ifndef TWIDDLE_hpp
#define TWIDDLE_hpp

#include <stdio.h>
#include <iostream>
#include <chrono>

#include "PID.h"

class TWIDDLE {

public:
  double m_p[3], m_dp[3];

  TWIDDLE(double dp[3], double p[3]);

  void run(PID &pid, double cte);

  bool getRestartSim();
  bool getTunningComplete();
  void printValues();


private:
  int m_step, m_pNum, m_count;
  double m_bestError, m_currentError;
  bool m_step_initialised;
  const double TOL;
  const int RESTART_VALUE;
  const int ACCUMSTART;
  const int MIN_MAX_ERROR;

  bool m_restartSim;
  bool m_tunningComplete;

  double m_cte;
  
  std::chrono::time_point<std::chrono::system_clock> now, prevTime;
  std::chrono::duration<double> timeDiff;

  // ---- FUNCTIONS ----
  void runInitialStep(PID &pid);
  void runPositiveAdjustment(PID &pid);
  void runNegativeAdjustment(PID &pid);
  void runPID(PID &pid);
  void checkTunningComplete();
};

#endif /* TWIDDLE_hpp */
