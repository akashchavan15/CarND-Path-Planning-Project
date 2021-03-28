#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;



int main() {
  uWS::Hub h;

  Helper helper;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // start in lane 1
  int host_car_ref_lane = 1;
  // have a reference velocity to target
  double host_car_ref_vel = 0; // mph
  
  h.onMessage([&helper,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&host_car_ref_vel,&host_car_ref_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = helper.verifyData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double host_car_x = j[1]["x"];
          double host_car_y = j[1]["y"];
          double host_car_s = j[1]["s"];
          double host_car_d = j[1]["d"];
          double host_car_yaw = j[1]["yaw"];
          double host_car_vel = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int previous_path_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
            
          // if previous path does contain points, assume car is at
          // end position of previous path
          if (previous_path_size > 0) {
            host_car_s = end_path_s;
          }
            
          /*
           * Criteria:
           * check if the car is too close to the vehicle in front of us
           * if true -> try to change lanes or slow dow
           * if false -> just stay in lane and match speed
           *  in order to determine whether we can change lanes:
           * is any of the two lanes left/right blocked by a car just behind us?
           * if (both) true -> slow down
           * if (any) false -> change lane
           * already initialized to take into account whether there actually
           * is a left lane / right lane
           *
           */
          bool change_lanes_or_slow_down = false;
          
          bool dont_go_left = (host_car_ref_lane == 0);
          bool dont_go_right = (host_car_ref_lane == 2);
          // changing lanes -> no double-lane-changes == too high accell!
          short host_car_lane = ((short)floor(host_car_d/4));
          bool changing_lanes = (host_car_lane != host_car_ref_lane);
          // distance of leading vehicle ahead
          double min_dist_left = 999.0;
          double min_dist_here = 999.0;
          double min_dist_right = 999.0;
            
          // find unit normal vector at currernt position
          int other_waypoint_idx = helper.getNextWaypoint(host_car_x, host_car_y, host_car_yaw, map_waypoints_x, 		                                                              map_waypoints_y);
          double other_waypoint_dx = map_waypoints_dx[other_waypoint_idx];
          double other_waypoint_dy = map_waypoints_dy[other_waypoint_idx];
            
          // find ref_v to use
          // go through all cars
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // check if another car is present in our lane
            float other_car_d = sensor_fusion[i][6];
            double other_car_vx = sensor_fusion[i][3];
            double other_car_vy = sensor_fusion[i][4];
            double other_car_v = sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
            double other_car_s = sensor_fusion[i][5];
            // calculate vd by dot product with d-vector
            double other_car_vd = other_car_vx*other_waypoint_dx + other_car_vy*other_waypoint_dy;
            // calculate vs by knowledge vs*vs + vd*vd = vx*vx + vy*vy
            double other_car_vs = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy - other_car_vd*other_car_vd);
            // project other cars position outwards based on its 
            // velocity times the distance at which we apply the first
            // action (end of current path)
            other_car_s += ((double)previous_path_size*.02*other_car_vs);
            // o_d += ((double)previous_path_size*.02*o_vd); // TODO accurate?
            short other_car_lane = ((short)floor(other_car_d/4));
            // what is the other car doing?
            bool other_car_is_going_right = (other_car_vd > 2.0);
            bool other_car_is_going_left  = (other_car_vd < -2.0);
            short other_car_direction     = other_car_is_going_right ? 1 : (other_car_is_going_left ? -1 : 0);
            short other_car_merging_lane  = other_car_lane + other_car_direction;
            // check if the car is in my lane or is (potentially) entering our lane
            bool other_car_is_in_my_lane  = (other_car_lane == host_car_ref_lane) || (other_car_merging_lane ==                                                         host_car_ref_lane);
            bool other_car_is_left  = (other_car_lane == host_car_ref_lane-1) || (other_car_merging_lane ==                                                       host_car_ref_lane-1);
            bool other_car_is_right = (other_car_lane == host_car_ref_lane+1) || (other_car_merging_lane ==                                                       host_car_ref_lane+1);
            // determine if car is 30m ahead, 15m behind or closer than 10m
            double other_car_distance = other_car_s-host_car_s;
            bool other_car_is_ahead  = (other_car_distance > 0.0) && (other_car_distance < 30.0);
            bool other_car_is_close  = abs(other_car_s-host_car_s) < 10;
            bool other_car_is_behind = (other_car_distance < 0.0) && (other_car_distance > -15.0);
            // update leading vehicle distance
            if (other_car_distance > 0.0) {
              if (other_car_is_in_my_lane) {
                if (other_car_distance < min_dist_here) min_dist_here = other_car_distance;
              } else if (other_car_is_left) {
                if (other_car_distance < min_dist_left) min_dist_left = other_car_distance;
              } else if (other_car_is_right) {
                if (other_car_distance < min_dist_right) min_dist_right = other_car_distance;
              }
            }
            // check if car is slower
            bool other_car_is_slower = other_car_v-host_car_ref_vel < 0;
            // is there a car in my lane ahead of us?
            // then we either have to switch lanes or decellerate
            change_lanes_or_slow_down = change_lanes_or_slow_down || (other_car_is_ahead && other_car_is_in_my_lane);
            // check if the car impedes my ability to swith lanes; if...
            // - it is closer than 30m ahead us and slower than us
            // - it is closer than 15m behind us and faster than us
            // - it is closer than 10m to us (better just wait until the situation becomes more clear)
            bool other_car_is_obstacle = ( (other_car_is_ahead && other_car_is_slower)
                                      || (other_car_is_behind && !other_car_is_slower)
                                      || other_car_is_close
                              );
            // don't switch lanes if the car is dangerous
            dont_go_left  = dont_go_left || (other_car_is_left && other_car_is_obstacle);
            dont_go_right = dont_go_right || (other_car_is_right && other_car_is_obstacle);
          }

          // don't switch lanes if leading car in target lane is closer than
          // leading car in current lane
          dont_go_left  = dont_go_left  || (min_dist_left < min_dist_here);
          dont_go_right = dont_go_right || (min_dist_right < min_dist_here);
          
          // if car too close is in front of car
          if (change_lanes_or_slow_down) {
            // if we are already changing lanes or cannot change 
            if (changing_lanes || (dont_go_left && dont_go_right) || host_car_ref_vel < 20) {
              // slow down instead (obeying max. accell. & velocity)
              if (host_car_ref_vel > 5)
                host_car_ref_vel -= .224;
            }
            else if (!dont_go_left) {
              // if left lane free, switch lane
             host_car_ref_lane = (host_car_ref_lane - 1);
            }
            else if (!dont_go_right) {
              // if right lane free, switch lane
             host_car_ref_lane = (host_car_ref_lane + 1);
            }
          } 
          else if (host_car_ref_vel < 49.5) {
            // if we are too slow, speed up (obeying max. accell. & velocity)
            host_car_ref_vel += .224;
          }

          // create a list of widely spaced (x,y) waypoints, evenly spaced at
          // 30m, later we will interpolate these waypoints with a spline and 
          // fill it in with more points
          vector<double> control_x;
          vector<double> control_y;

          // reference x,y,yaw states: the state where we can actually control
          // the car right now; either the current state or the end of the
          // path calculated in the previous round (kept in order to smooth
          // the trajectory
          double host_car_ref_x = host_car_x;
          double host_car_ref_y = host_car_y;
          double ref_yaw = helper.deg2rad(host_car_yaw);

          if (previous_path_size < 2) {
            // if previous size is almost empty, use the car as starting ref
            // but add one point behind it to make the path tangent to the car
            double host_car_prev_x = host_car_x - cos(host_car_yaw);
            double host_car_prev_y = host_car_y - sin(host_car_yaw);
            control_x.push_back(host_car_prev_x);
            control_x.push_back(host_car_x);
            control_y.push_back(host_car_prev_y);
            control_y.push_back(host_car_y);
          }
          else {
            // redefine reference state as previous path and point
            host_car_ref_x = previous_path_x[previous_path_size-1];
            host_car_ref_y = previous_path_y[previous_path_size-1];
            double host_car_prev_x = previous_path_x[previous_path_size-2];
            double host_car_prev_y = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(host_car_ref_y-host_car_prev_y, host_car_ref_x-host_car_prev_x);
            // use two points that make the path tangent to the previous 
            // end point
            control_x.push_back(host_car_prev_x);
            control_x.push_back(host_car_ref_x);
            control_y.push_back(host_car_prev_y);
            control_y.push_back(host_car_ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting ref
          for (int spacing = 30; spacing <= 90; spacing += 30) {
            vector<double> controlpoint = helper.getXY(host_car_s+spacing, (2+4*host_car_ref_lane), 
                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
            control_x.push_back(controlpoint[0]);
            control_y.push_back(controlpoint[1]);
          }

          // transform controlpoints to car coordinate starting from 
          // reference position
          vector<double> control_x_car;
          vector<double> control_y_car;

          for (int i = 0; i < static_cast<int>(control_x.size()); i++) {
            // shift
            double x_shifted = control_x[i]-host_car_ref_x;
            double y_shifted = control_y[i]-host_car_ref_y;
            // rotate
            control_x_car.push_back(x_shifted*cos(0-ref_yaw)-y_shifted*sin(0-ref_yaw));
            control_y_car.push_back(x_shifted*sin(0-ref_yaw)+y_shifted*cos(0-ref_yaw));
          }

          // create a spline & set (x,y) points to the spline
          tk::spline spline_car;
          spline_car.set_points(control_x_car, control_y_car);

          // sample (x,y) points from spline
          vector<double> host_car_next_x;
          vector<double> host_car_next_y;

          // start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            host_car_next_x.push_back(previous_path_x[i]);
            host_car_next_y.push_back(previous_path_y[i]);
          }

          // calculate with which distance to interpolate along the spline 
          // so we travel at our desired reference velocity
          // path will extend 30m ahead in x direction
          double target_x_car = 30.0;
          // get y value for that distance
          double target_y_car = spline_car(target_x_car);
          // calculate euclidean distance travelled to that point
          double target_dist = sqrt(target_x_car*target_x_car + target_y_car*target_y_car);
          // calculate spacing of points needed to find reference velocity
          double num_points = target_dist / (.02*host_car_ref_vel/2.24);

          // fill up the rest of our path planner so we have 50 points
          double recent_x_car = 0;
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            double x_car = recent_x_car + target_x_car/num_points;
            double y_car = spline_car(x_car);
            recent_x_car = x_car;
            // transform car coordinates to world coordinates
            // rotate
            double x = (x_car*cos(ref_yaw)-y_car*sin(ref_yaw));
            double y = (x_car*sin(ref_yaw)+y_car*cos(ref_yaw));
            // shift
            x += host_car_ref_x;
            y += host_car_ref_y;
            // add to return value
            host_car_next_x.push_back(x);
            host_car_next_y.push_back(y);
          }

          json msgJson;
          msgJson["next_x"] = host_car_next_x;
          msgJson["next_y"] = host_car_next_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } 
      else {
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
