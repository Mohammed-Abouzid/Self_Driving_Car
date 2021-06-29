#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <cmath>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  // preset
  int lane = 1;             // start in lane 1 (in the middle!);
  double ref_vel = 0.0;     // actual reference velocity, start with 0 mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // *****> lane and ref_vel must be mentioned in [] to avoid a lambda error...
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"]; // The car's x position in map coordinates
          double car_y = j[1]["y"]; // The car's y position in map coordinates
          double car_s = j[1]["s"]; // The car's s position in frenet coordinates
          double car_d = j[1]["d"]; // The car's d position in frenet coordinates
          double car_yaw = j[1]["yaw"]; // The car's yaw angle in the map
          double car_speed = j[1]["speed"]; // The car's speed in MPH

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"]; // The previous list of x points previously given to the simulator
          auto previous_path_y = j[1]["previous_path_y"]; // The previous list of y points previously given to the simulator
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"]; // The previous list's last point's frenet s value
          double end_path_d = j[1]["end_path_d"]; // The previous list's last point's frenet d value

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"]; // A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 
          
          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          //   ####################   START   ##############################
          // get the cars position
          if(previous_path_x.size() > 0){car_s = end_path_s;}
          
          // get the free lanes and if too close to the car in front of us
          vector<bool> free_lane__too_close = get_fusion(sensor_fusion, previous_path_x, car_s, lane);   // get the free lanes [0,1,2] and if too close to the front car [3]
          vector<bool> free_lane = vector<bool>(free_lane__too_close.begin(), free_lane__too_close.end()-1);
          bool too_close = free_lane__too_close.back();

          //cout << "Free Lane: ** "<< free_lane[0] << " ** " << free_lane[1] << " ** " << free_lane[2] << endl; 
          //cout << "too close: " << too_close << "veloc: " << ref_vel << endl;

          // update/change the lane and the velocity
          // check the right lane
          if(too_close == true){
            if( (lane+1 <= 2) && free_lane[lane+1] )           
              lane++;      
            // check the left lane
            else if ((lane-1>=0) && free_lane[lane-1])   
              lane--;  
            // dont change the lane and reduce the speed
            else                      
              ref_vel -= 0.224;        
          }
          else if (ref_vel < 49.5){ref_vel += 0.224;} 
          
          // in Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
          vector<double> ptsx;
          vector<double> ptsy;
          double x_ref;
          double y_ref;
          double yaw_ref;
          
          // get the waypoints
          get_points(car_x, car_y, car_yaw, next_wp0, next_wp1, next_wp2, previous_path_x, previous_path_y, &ptsx, &ptsy, &yaw_ref, &x_ref, &y_ref);
          

          //create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points from the last time - add them to the path planner!
          for(int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired refrence velocity
          double target_x = 30.0;
          double target_y = s(target_x);  
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on = 0; 

          // Fill up the rest of our path planner after filling it with previous points,  we are looking at maximum 50 points
          for (int i=1; i<= 50-previous_path_x.size(); i++){ 
            double N = (target_dist/(0.02*ref_vel/2.24));   
            double x_point = x_add_on+(target_x)/N;    
            double y_point = s(x_point);        
            x_add_on = x_point;

            double ref_X = x_point;
            double ref_y = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (ref_X * cos(yaw_ref)-ref_y*sin(yaw_ref));
            y_point = (ref_X * sin(yaw_ref)+ref_y*cos(yaw_ref));

            x_point += x_ref;
            y_point += y_ref;
            
            // push the x and y coordinates back
            next_x_vals.push_back(x_point); 
            next_y_vals.push_back(y_point);
          }

          //  ######################      END      ##############################

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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