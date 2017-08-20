#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "TWIDDLE.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  // Set up of Twiddle auto tunning
  double initial_dp[3] = {0.01,0.00005,0.001};
  //double initial_p[3] = {-0.09, 0.0, 0.00209032};
  double initial_p[3] = {-0.065,0.001,-0.5};
  TWIDDLE twiddle(initial_dp, initial_p);
  
  bool is_PIDTunning = false;
  pid.Init(initial_p[0],initial_p[1],initial_p[2]);

  
  h.onMessage([&pid,&twiddle,&is_PIDTunning](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          if (is_PIDTunning)
          {
            
            twiddle.run(pid, cte);
            if (twiddle.getRestartSim()) // restart the simulator
            {
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              std::cout << "RESET!" << std::endl;
            }
            
            // exit if the tunning is complete
            if (twiddle.getTunningComplete())
            {
              std::cout << "TUNNING COMPLETE" << std::endl;
              std::cout << " Final Paramaters" << std::endl;
              std::cout << "Kp: " << twiddle.m_p[0] << " Ki: " << twiddle.m_p[1] << " Kd: " << twiddle.m_p[2] << std::endl;
              is_PIDTunning = false;
              pid.Init(twiddle.m_p[0], twiddle.m_p[1], twiddle.m_p[2]);
            }
            twiddle.printValues();
          }
          else
          {
            pid.UpdateError(cte);
            pid.CalcOutput();
          }
          steer_value = pid.currentOutput;
//
//          std::cout << "step: " << twiddle.step << " count: " << twiddle.count << " pNum: " << twiddle.pNum << " BestError: " << twiddle.bestErr << " CurrentError: " << twiddle.newError << std::endl;
//          std::cout << " Kp,Ki,Kd: " << twiddle.p[0] << "," << twiddle.p[1] << "," << twiddle.p[2] << " dp: " << twiddle.dp[0] << "," << twiddle.dp[1] << "," << twiddle.dp[2] << std::endl;
//          twiddle.count++;
//          // At initial we just want to run and accumulate the error
//          if (twiddle.step == 0)
//          {
//            pid.UpdateError(cte);
//            steer_value = pid.Kp * pid.p_error + pid.Ki * pid.i_error + pid.Kd * pid.d_error;
//            twiddle.bestErr += cte*cte;
//            
//            if (twiddle.count == twiddle.RESTART_VALUE)
//            {
//              twiddle.count = 0;
//              std::string msg = "42[\"reset\",{}]";
//              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//              std::cout << "RESET!" << std::endl;
//              
//              // Move to the next step
//              twiddle.step++;
//            }
//          }
//          else if (twiddle.step == 1)
//          {
//            if (!twiddle.step_initialised)
//            {
//              twiddle.p[twiddle.pNum] += twiddle.dp[twiddle.pNum];
//              pid.Init(twiddle.p[0], twiddle.p[1], twiddle.p[2]);
//              twiddle.step_initialised = true;
//              // Clear the error for the new step
//              twiddle.newError = 0;
//            }
//            pid.UpdateError(cte);
//            steer_value = pid.Kp * pid.p_error + pid.Ki * pid.i_error + pid.Kd * pid.d_error;
//            twiddle.newError += cte*cte;
//            
//            // Stop the run if time out or already surpassed the current best
//            if (twiddle.count == twiddle.RESTART_VALUE || twiddle.newError > twiddle.bestErr)
//            {
//              twiddle.count = 0;
//              std::string msg = "42[\"reset\",{}]";
//              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//              std::cout << "RESET!" << std::endl;
//              
//              // See if there is an improvement
//              if (twiddle.newError < twiddle.bestErr)
//              {
//                twiddle.bestErr = twiddle.newError;
//                twiddle.dp[twiddle.pNum] *= 1.1;
//                if (twiddle.pNum == 2) // finished with D term - check if we should exit
//                {
//                  double sum = 0;
//                  for (int i=0; i<3; i++)
//                  {
//                    sum += twiddle.dp[i];
//                  }
//                  if (sum < twiddle.tol)
//                  {
//                    std::cout << "Tuning Complete" << std::endl;
//                    return 0; // Exit
//                  }
//                }
//                // Increment to the next variable to tune
//                if (twiddle.pNum == 2) // Go back to the first variable if at D else move on
//                  twiddle.pNum = 0;
//                else
//                  twiddle.pNum++;
//                twiddle.step_initialised = false;
//              }
//              else
//              {
//                // Move to the next step
//                twiddle.step++;
//                twiddle.step_initialised = false;
//              }
//              
//            }
//          }
//          else if (twiddle.step == 2)
//          {
//            if (!twiddle.step_initialised)
//            {
//              twiddle.p[twiddle.pNum] -= twiddle.dp[twiddle.pNum]*2;
//              pid.Init(twiddle.p[0], twiddle.p[1], twiddle.p[2]);
//              twiddle.step_initialised = true;
//              // Clear the error for the new step
//              twiddle.newError = 0;
//            }
//            pid.UpdateError(cte);
//            steer_value = pid.Kp * pid.p_error + pid.Ki * pid.i_error + pid.Kd * pid.d_error;
//            twiddle.newError += cte*cte;
//            
//            // Stop the run if time out or already surpassed the current best
//            if (twiddle.count == twiddle.RESTART_VALUE || twiddle.newError > twiddle.bestErr)
//            {
//              twiddle.count = 0;
//              std::string msg = "42[\"reset\",{}]";
//              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//              std::cout << "RESET!" << std::endl;
//              
//              if (twiddle.newError < twiddle.bestErr)
//              {
//                twiddle.bestErr = twiddle.newError;
//                twiddle.dp[twiddle.pNum] *= 1.1;
//                if (twiddle.pNum == 2) // finished with D term - check if we should exit
//                {
//                  double sum = 0;
//                  for (int i=0; i<3; i++)
//                  {
//                    sum += twiddle.dp[i];
//                  }
//                  if (sum < twiddle.tol)
//                  {
//                    std::cout << "Tuning Complete" << std::endl;
//                    return 0; // Exit
//                  }
//                }
//                else
//                {
//                  // Increment to the next variable to tune
//                  twiddle.pNum++;
//                  twiddle.step = 1;
//                  twiddle.step_initialised = false;
//                }
//              }
//              else
//              {
//                // Go back to step one for the new variable, reduce the change in p
//                twiddle.step = 1;
//                twiddle.step_initialised = false;
//                // Remove the changes that where made
//                twiddle.p[twiddle.pNum] += twiddle.dp[twiddle.pNum];
//                twiddle.dp[twiddle.pNum] *= 0.9;
//                if (twiddle.pNum == 2) // Go back to the first variable if at D else move on
//                  twiddle.pNum = 0;
//                else
//                  twiddle.pNum++;
//              }
//            }
//          }
          
//          count++;
//          std::cout << "COUNT: " << count << std::endl;
//          if(count == 1500)
//          {
//            count = 0;
//            std::string msg = "42[\"reset\",{}]";
//            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//            std::cout << "RESET!" << std::endl;
//          }
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
