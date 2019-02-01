#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
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
    // reinit state
    mse = 0.0;
    best_error = 0.0;

    // update params
    double dir = d_dir[dir_idx];
    params[param_idx] += dir * d[param_idx];

    // re-create PID Controller
    steer_pid = PID(params[0], params[1], params[2]);

    // update d
    dir_idx++;
    if (dir_idx > max_dir_idx) {
      dir_idx = 0;
      param_idx = (param_idx + 1) % max_param_idx;
    }

  }

  bool stage_completed() {
    return cur_iter == num_iter;
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

  const int num_iter = 500;
  // const int num_iter = 4500; // full loop at 20 MPH
  int cur_iter = 0;

  const int max_param_idx = params.size();
  int param_idx = 0; // index of a paremeter to tune

  const int max_dir_idx = d_dir.size();
  int dir_idx = 0;

  double best_error = 0.0;

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
          std::cout << "Iter: " << ctx.cur_iter << std::endl;

          if (ctx.stage_completed()) {
            ctx.next_stage();
          }

          
          ctx.steer_pid.UpdateError(cte);
          double steer_value = -ctx.steer_pid.TotalError();



          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          string msg;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          msg = "42[\"steer\"," + msgJson.dump() + "]";

          // if (speed >= 19) {
            // msg = "42[\"reset\",{}]";
          // }

          std::cout << msg << std::endl;
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
    std::cout << "Connected!!!" << std::endl;
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