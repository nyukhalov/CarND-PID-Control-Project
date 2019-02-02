#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <stdio.h>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

class Context {
public:

  void next_stage() {
    stage_no++;

    double d_sum = 0;
    for(int i=0; i<d.size(); i++) d_sum += d[i];

    cout << left
         << "stage_no=" << setw(5) << stage_no 
         << ", d_sum=" << setw(8) << d_sum 
         << ", best_err=" << setw(8) << best_error 
         << ", MSE=" << setw(9) << get_mse() 
         << ", p={" 
          << setw(5)<< params[0] << ", " 
          << setw(5)<< params[1] << ", " 
          << setw(5)<< params[2] << "}"
         << ", d={" 
          << setw(5) << d[0] << ", " 
          << setw(5) << d[1] << ", " 
          << setw(5) << d[2] << "}" 
        << std::endl;

    double err = get_mse();
    if (best_error < 0 || err < best_error) 
    {
      best_error = err;
      dir_idx = 0;
      
      // it is not the first run
      if (best_error >= 0) {
        d[param_idx] *= 1.1;
        param_idx = (param_idx + 1) % max_param_idx;
      }
    }
    else
    {
      dir_idx++;
      if (dir_idx >= max_dir_idx - 1)
      {
        double dir = d_dir[dir_idx];
        params[param_idx] += dir * d[param_idx];
        param_idx = (param_idx + 1) % max_param_idx;  
        dir_idx = 0;
      }
    }

    // reinit state
    mse = 0.0;
    cur_iter = 0;

    // update params
    double dir = d_dir[dir_idx];
    params[param_idx] += dir * d[param_idx];

    // re-create PID Controller
    steer_pid = PID(params[0], params[1], params[2]);
  }

  bool stage_completed() {
    return cur_iter == num_iter;
  }

  void update_mse(double cte) {
    mse += cte*cte;
  }

  double get_mse() {
    return mse / cur_iter;
  }

  double mse = 0.0;

  const double desired_speed = 20;

  // [Kp, Ki, Kd]
  vector<double> params = {0.1, 0, 0.1};
  vector<double> d = {1, 1, 1};
  vector<double> d_dir = {1, -2, 1};

  int stage_no = 0;

  const int num_iter = 500;
  // const int num_iter = 4500; // full loop at 20 MPH
  int cur_iter = 0;

  const int max_param_idx = params.size();
  int param_idx = 0; // index of a paremeter to tune

  const int max_dir_idx = d_dir.size();
  int dir_idx = 0;

  double best_error = -1; // initial state

  PID steer_pid = PID(0.1, 0, 0.1);
};

int main() {
  uWS::Hub h;

  Context ctx;

  PID speed_pid(0.3, 0, 0);

  h.onMessage([&ctx, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());

          double speed_cte = speed - ctx.desired_speed;
          speed_pid.UpdateError(speed_cte);
          double throttle = -speed_pid.TotalError();
          
          ctx.cur_iter++;
          // std::cout << "iter=" << ctx.cur_iter << ", MSE=" << ctx.get_mse() << std::endl;

          ctx.steer_pid.UpdateError(cte);
          double steer_value = -ctx.steer_pid.TotalError();
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          ctx.update_mse(cte);

          bool reset = false;
          if (ctx.stage_completed()) {
            //std::cout << "Stage completed. Reset state. Number of iterations per run=" << ctx.num_iter << std::endl;
            reset = true;
            ctx.next_stage();
          }
          
          string msg;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          msg = "42[\"steer\"," + msgJson.dump() + "]";

          if (reset) {
            msg = "42[\"reset\",{}]";
          }

          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    // std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}