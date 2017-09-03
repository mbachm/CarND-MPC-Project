#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "utils.h"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/**
 * convertWaypoints Converts the waypoints from the map coordinate system to the car coordinate system.
 * @param ptsx vector<double> with x values of the values to find a fitting polynomial.
 * @param ptsy vector<double> with y values of the values to find a fitting polynomial.
 * @param px double value of current x-position of the car on global map.
 * @param py double value of current y-position of the car on global map.
 * @param psi double value of current psi.
 * @output A vector<Eigen::VectorXd> vectors containing the x and y values of the waypoints in vehicle coordinates.
 */
vector<Eigen::VectorXd> convertWaypoints(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi) {
  assert(ptsx.size() == ptsy.size());
  const int size = ptsx.size();
  Eigen::VectorXd ptsx_local(size);
  Eigen::VectorXd ptsy_local(size);
  
  for (size_t i = 0; i < ptsx.size(); i++) {
    ptsx_local[i] = (ptsx[i] - px)*cos(-psi) - (ptsy[i] - py)*sin(-psi);
    ptsy_local[i] = (ptsx[i] - px)*sin(-psi) + (ptsy[i] - py)*cos(-psi);
  }
  return {ptsx_local, ptsy_local};
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          // As mentioned in the "Tips and Tricks for the MPC Project" section, steer_value has to be
          // mulitplied by -1 for the simulator
          delta *= -1.0;
          double acceleration = j[1]["throttle"];
          
          // Adding latency of 100 ms into the model
          // See https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/4
          const double latency = 0.1;
          px = px + v*cos(psi)*latency;
          py = py + v*sin(psi)*latency;
          psi = psi + v*delta/Lf*latency;
          v = v + acceleration*latency;
          
          
          //Convert waypoints to car coordinate system
          vector<Eigen::VectorXd> local_waypoints = convertWaypoints(ptsx, ptsy, px, py, psi);
          Eigen::VectorXd ptsx_local = local_waypoints[0];
          Eigen::VectorXd ptsy_local = local_waypoints[1];
          
          // const for local coordinates
          const double px_local = 0.0, py_local = 0.0, psi_local = 0.0;
          
          auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
          
          //Calculating errors
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          // cte = polyeval(coeffs, x) - y;
          double cte = polyeval(coeffs, px_local) - py_local;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = psi_local - atan(polyderivativeeval(coeffs, px_local));
          
          Eigen::VectorXd state(6);
          state << px_local, py_local, psi_local, v, cte, epsi;
          auto solution = mpc.Solve(state, coeffs);
          
          /*
           * Both are in between [-1, 1].
           */
          const double steer_value = solution[0][0];
          const double throttle_value = solution[0][1];
          std::cout << "steer_value " << steer_value << std::endl;
          std::cout << "throttle_value " << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // As mentioned in the "Tips and Tricks for the MPC Project" section, steer_value has to be
          // mulitplied by -1 for the simulator
          msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          const vector<double>& mpc_x_vals = solution[1];
          const vector<double>& mpc_y_vals = solution[2];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          //Display the waypoints/reference line
          vector<double> next_x_vals(ptsx.size());
          vector<double> next_y_vals(ptsy.size());
          
          for (size_t i = 0; i < ptsx_local.size(); ++i) {
            next_x_vals[i] = ptsx_local[i];
            next_y_vals[i] = ptsy_local[i];
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
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
