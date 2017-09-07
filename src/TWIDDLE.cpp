#include "TWIDDLE.h"

TWIDDLE::TWIDDLE(double dp[3], double p[3]) :
  m_step(0),
  m_pNum(0),
  m_count(0),
  m_bestError(0.0),
  m_currentError(0.0),
  m_step_initialised(false),
  TOL(0.005),
  RESTART_VALUE(4000),
  ACCUMSTART(100),
  MIN_MAX_ERROR(80000),
  m_restartSim(false),
  m_cte(0.0)
{
  for (int i=0; i<3; i++)
  {
    m_p[i] = p[i];
    m_dp[i] = dp[i];
  }
}

void TWIDDLE::run(PID &pid, double cte)
{
  m_cte = cte;
  m_count++;
  switch (m_step) {
    // At step one, run to get initial error
    case 0:
      runInitialStep(pid);
      break;
    case 1:
      runPositiveAdjustment(pid);
      break;
    case 2:
      runNegativeAdjustment(pid);
      break;
    default:
      break;
  }
}

bool TWIDDLE::getRestartSim()
{
  return m_restartSim;
}

bool TWIDDLE::getTunningComplete()
{
  return m_tunningComplete;
}

void TWIDDLE::printValues()
{
  std::cout << "step: " << m_step << " count: " << m_count << " pNum: " << m_pNum << " BestError: " << m_bestError << " CurrentError: " << m_currentError << std::endl;
  std::cout << " Kp,Ki,Kd: " << m_p[0] << "," << m_p[1] << "," << m_p[2] << " dp: " << m_dp[0] << "," << m_dp[1] << "," << m_dp[2] << std::endl;
}

void TWIDDLE::runPID(PID &pid)
{
  pid.UpdateError(m_cte);
  pid.CalcOutput();
}

void TWIDDLE::runInitialStep(PID &pid)
{
  if (!m_step_initialised)
  {
    pid.Init(m_p[0], m_p[1], m_p[2]);
    m_step_initialised = true;
  }
  runPID(pid);
  if (m_count >= ACCUMSTART)
    m_bestError += m_cte*m_cte;

  if (m_count > RESTART_VALUE)
  {
    // Restart the sim and move to the next step
    m_restartSim = true;
    m_step++;
    m_step_initialised = false;
    m_count = 0;
    
    // Limit bestError to make sure bad results dont accidentally succeed
    if(m_bestError > MIN_MAX_ERROR)
      m_bestError = MIN_MAX_ERROR;
  }

}

void TWIDDLE::runPositiveAdjustment(PID &pid)
{
  if (!m_step_initialised)
  {
    m_restartSim = false;
    // increase the coefficient
    m_p[m_pNum] += m_dp[m_pNum];
    pid.Init(m_p[0], m_p[1], m_p[2]);
    m_step_initialised = true;
    m_currentError = 0.0;
  }
  runPID(pid);
  if (m_count >= ACCUMSTART)
    m_currentError += m_cte*m_cte;

  // Restart if timed out or error is already larger than the best error
  if (m_count > RESTART_VALUE || m_currentError > m_bestError)
  {
    m_count = 0;
    m_restartSim = true;

    if (m_currentError < m_bestError) // If the error has improved increase dp
    {
      m_bestError = m_currentError;
      m_dp[m_pNum] *= 1.1;

      if (m_pNum == 2) // If d term
      {
        checkTunningComplete();
        m_pNum = 0; // go back to the P term
      }
      else
      {
        m_pNum++;
      }
    }
    else // If worsened move to the negative step
    {
      m_step++;
    }
    m_step_initialised = false;
  }
}

void TWIDDLE::runNegativeAdjustment(PID &pid)
{
  if (!m_step_initialised)
  {
    m_restartSim = false;
    // decrease the coefficient (x2 to remove effect of positive adjustment)
    m_p[m_pNum] -= m_dp[m_pNum]*2;
    pid.Init(m_p[0], m_p[1], m_p[2]);
    m_step_initialised = true;
    m_currentError = 0.0;
  }
  runPID(pid);
  if (m_count >= ACCUMSTART)
    m_currentError += m_cte*m_cte;

  if (m_count > RESTART_VALUE || m_currentError > m_bestError)
  {
    m_count = 0;
    m_restartSim = true;

    if (m_currentError < m_bestError) // If the error has improved increase dp
    {
      m_bestError = m_currentError;
      m_dp[m_pNum] *= 1.1;

      if (m_pNum == 2) // If d term
      {
        checkTunningComplete();
        m_pNum = 0; // go back to the P term
      }
      else
      {
        m_pNum++;
      }
    }
    else // If worsened decrease dp
    {
      // Remove the changes to p
      m_p[m_pNum] += m_dp[m_pNum];
      m_dp[m_pNum] *= 0.9;

      // move onto the next coefficient
      if (m_pNum == 2)
        m_pNum = 0;
      else
        m_pNum++;
    }
    m_step = 1; // return to step one with the next coefficient
    m_step_initialised = false;
  }
}

void TWIDDLE::checkTunningComplete()
{
  double sum = 0;
  for (int i=0; i<3; i++)
  {
    sum += m_dp[i];
  }
  if (fabs(sum) < TOL)
    m_tunningComplete = true;
}
