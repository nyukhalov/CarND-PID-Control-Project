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

class Twiddle {
public:

  Twiddle(int num_iter, int num_iter_ignore)
  : num_iter(num_iter), num_iter_ignore(num_iter_ignore)
  {}

  void next_stage() {
    stage_no++;

    double d_sum = 0;
    for(int i=0; i<d.size(); i++) d_sum += d[i];

    cout << left
         << "stage_no=" << setw(5) << stage_no 
         << ", d_sum=" << setw(9) << d_sum 
         << ", best_err=" << setw(9) << best_error 
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
      // it is not the first run
      if (best_error >= 0) {
        d[param_idx] *= 1.1;
        param_idx = (param_idx + 1) % max_param_idx;
      }

      best_error = err;
      dir_idx = 0;
    }
    else
    {
      dir_idx++;
      if (dir_idx >= max_dir_idx - 1)
      {
        double dir = d_dir[dir_idx];
        params[param_idx] += dir * d[param_idx];
        d[param_idx] *= 0.9;

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

  void force_stop_stage() {
    cur_iter = num_iter;
    mse = fabs(best_error * num_iter * 2); // must be always bigger than best_error
  }

  void update_mse(double cte) {
    if (cur_iter > num_iter_ignore) {
      mse += cte*cte;
    }
  }

  double get_mse() {
    return mse / cur_iter;
  }

  int cur_iter = 0;

  PID steer_pid = PID(0.12, 0.0005, 3.8);

private:
  const int num_iter;
  const int num_iter_ignore;

  // [Kp, Ki, Kd]
  vector<double> params = {0.12, 0.0005, 3.8};
  vector<double> d = {0.1, 0.0005, 1};
  vector<double> d_dir = {1, -2, 1};

  int stage_no = 0;
  double mse = 0.0;

  double best_error = -1; // initial state

  const int max_dir_idx = d_dir.size();
  int dir_idx = 0;  

  const int max_param_idx = params.size();
  int param_idx = 0; // index of a paremeter to tune  
};

int main() {
  uWS::Hub h;

  const int num_iter = 3000;
  // const int num_iter = 7000;
  const int num_iter_ignore = 200;
  Twiddle twiddle(num_iter, num_iter_ignore);

  int stuck_counter = 0;
  const double desired_speed = 30;
  PID speed_pid(0.9, 0.0001, 0);

  double lowpass_gain = .3;
  double prev_steer_angle = 0;  

  h.onMessage([&twiddle, &speed_pid, desired_speed, &stuck_counter, lowpass_gain, &prev_steer_angle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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

          if (speed <= 2) stuck_counter++;
          if (stuck_counter >= 500) {
            stuck_counter = 0;
            twiddle.force_stop_stage();
          }

          bool reset = false;
          if (twiddle.stage_completed()) {
            reset = true;
            twiddle.next_stage();
          }

          double speed_cte = speed - desired_speed;
          speed_pid.UpdateError(speed_cte);
          double throttle = -speed_pid.TotalError();
          
          twiddle.cur_iter++;
          // std::cout << twiddle.cur_iter << std::endl;

          twiddle.steer_pid.UpdateError(cte);
          double steer_value = -twiddle.steer_pid.TotalError();

          if (steer_value > 1) steer_value = 1;
          else if (steer_value < -1) steer_value = -1;

          double g = 1 - lowpass_gain;
          steer_value = lowpass_gain*steer_value + g*prev_steer_angle;

          prev_steer_angle = steer_value;

          twiddle.update_mse(cte);
          
          string msg;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          msg = "42[\"steer\"," + msgJson.dump() + "]";

          if (reset) {
            msg = "42[\"reset\",{}]";
          }

          // std::cout << msg << std::endl;
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