#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


// get the free lanes and if we are too close to the car in front  ######
vector<bool> get_fusion(const vector<vector<double>> sensor_fusion, const vector<double> previous_path_x, double car_s, const int lane){
  
  const double speed_thre = 30;
  // check all cars on the map
  vector<bool> free_lane__too_close;         
  free_lane__too_close = {true, true, true, false};     // the first 3: is the lane free     // the last: is the front car too close
  
  for(int i=0; i < sensor_fusion.size(); i++){                  
    float d = sensor_fusion[i][6];      

    double check_lane;      
    if (d > 0 && d < 4) check_lane=0;  
    else if (d > 4 && d < 8) check_lane=1;  
    else check_lane=2;                 
    // get the cars velocity
    double v_x = sensor_fusion[i][3]; 
    double v_y = sensor_fusion[i][4]; 
    // check the speed and the distance to the front car
    double check_speed = sqrt(v_x*v_x+v_y*v_y);
    double check_car_s = sensor_fusion[i][5];
    check_car_s += ((double) previous_path_x.size() * 0.02 * check_speed); 
    // check if too close to the car in front
    if( d < (2+4*lane+2) && d > (2+4*lane-2) ){ 
      if((check_car_s > car_s) && ((check_car_s-car_s) < speed_thre) ){free_lane__too_close[3] = true;}
    }

    if( check_car_s > car_s && (check_car_s-car_s) < (speed_thre/1.2) ){free_lane__too_close[check_lane] = false;} 
    
    if( check_car_s < car_s && (car_s-check_car_s) < (speed_thre/3) ){free_lane__too_close[check_lane] = false;}   
    
  }
  
  return free_lane__too_close; 
}



// get the waypoints #######
void get_points(const double car_x, const double car_y, const double car_yaw, const vector<double> next_wp0, const vector<double> next_wp1, const vector<double> next_wp2, const vector<double> previous_path_x, const vector<double> previous_path_y, vector<double>* ptsx, vector<double>* ptsy, double* yaw_ref, double* x_ref, double* y_ref)
{
  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 20m
  // Later we will interoplate these waypoints with a spline and fill it in with more points that control speed
  vector<double> ptsx_;
  vector<double> ptsy_;

  // reference x,y, yaw states
  // eithe raw will reference the starting point as where the car is or at the previous paths end point
  double x_ref_ = car_x;
  double y_ref_ = car_y;
  double yaw_ref_ = deg2rad(car_yaw);

  // if previous size is almost empty, use the car as starting reference
  if(previous_path_x.size() <2)
  {
    // Use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw); 

    // generate two points the have a tangent path
    ptsx_.push_back(prev_car_x);
    ptsx_.push_back(car_x);

    ptsy_.push_back(prev_car_y);
    ptsy_.push_back(car_y);
  }
  // use the previous paths end point as starting reference
  else{
    // Refinde reference state as previous path end point
    x_ref_ = previous_path_x[previous_path_x.size()-1];
    y_ref_ = previous_path_y[previous_path_x.size()-1];

    double x_ref_prev = previous_path_x[previous_path_x.size()-2];
    double y_ref_prev = previous_path_y[previous_path_x.size()-2];
    yaw_ref_ = atan2(y_ref_-y_ref_prev, x_ref_-x_ref_prev);

    // Use two points that make the path tangent to the previous paths end point
    ptsx_.push_back(x_ref_prev);
    ptsx_.push_back(x_ref_);

    ptsy_.push_back(y_ref_prev);
    ptsy_.push_back(y_ref_);
  }


  // in Frenet add evenly 30m spaced points ahead of the starting reference
  ptsx_.push_back(next_wp0[0]);
  ptsx_.push_back(next_wp1[0]);
  ptsx_.push_back(next_wp2[0]);
  ptsy_.push_back(next_wp0[1]);
  ptsy_.push_back(next_wp1[1]);
  ptsy_.push_back(next_wp2[1]);

  for(int i=0; i<ptsx_.size(); i++){
    // shift car reference angle to 0 degree
    double shift_x = ptsx_[i]-x_ref_;
    double shift_y = ptsy_[i]-y_ref_;

    ptsx_[i] = (shift_x * cos(0-yaw_ref_)-shift_y*sin(0-yaw_ref_));
    ptsy_[i] = (shift_x * sin(0-yaw_ref_)+shift_y*cos(0-yaw_ref_));
  }
  
  *x_ref = x_ref_;
  *y_ref = y_ref_;
  *yaw_ref = yaw_ref_;
  *ptsx = ptsx_;
  *ptsy = ptsy_;
}




#endif  // HELPERS_H