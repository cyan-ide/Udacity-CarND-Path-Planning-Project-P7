#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
    
  // Start in lane 1
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 0.0; //49.5; // mph
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

            /**
             * TODO: define a path made up of (x,y) points that the car will visit
             *   sequentially every .02 seconds
             */

            
          // Provided previous path point size
          int prev_size = previous_path_x.size();
            
          //list of points in x-y coord system to add into the path (this will be used for interpolating spline)
          vector<double> ptsx;
          vector<double> ptsy;
            
          // Reference x, y, and yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
            
            
          // account for cars on the road
          if(prev_size > 0) //car starting position is last of the path
          {
              car_s = end_path_s;
          }

          bool too_close = false;
            double front_car_speed = -1;
          vector<bool> lane_taken = {false, false, false}; //for each lane keep flag if its safe to change or not
          // iterate over all objects on the road (sensor_fusion data) and check for possible collisions
            
            for (int i = 0; i < sensor_fusion.size(); i++) {
                float check_object_d = sensor_fusion[i][6]; // d value = tell object position horizontally (ie. on which lane)
                double check_object_s = sensor_fusion[i][5]; //s value = tell object position vertically (ie. how far along the road, used to check if object position is ahead / behind our car)
                //check the speed on the object
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);

                // dummy way to estimate objects position in future = assume will keep same speed and multiply by amount of time stamps similar as for our own car (ie. one simulator time stamp is 0.02s)
                // = 0.02 * (our own car prediction length from previous iteration) * current_object_speed
                check_object_s += ((double)prev_size*.02*check_speed);
                
                //object lane
                int check_lane = -1;
                //calculate object lane based on d value

                for (int lane_no = 0; lane_no < 3; lane_no++) // iterate 3 possible lanes = {0,1,2}
                {
                    if ( check_object_d< (2+4*lane_no+2) && check_object_d > (2+4*lane_no-2) ) {
                        check_lane = lane_no;
                    }
                }
                
                if (check_lane <0) { //some reported objects in sensor_fusion apparently can be outside of the three lanes, igonore them
                    continue;
                }
                
                //check which lanes are taken
                if (   (car_s - check_object_s  < 30)
                      && (check_object_s - car_s < 30 ))     //if object is 30meters behind or less OR object is 30 meters ahead or less
                {
                    lane_taken[check_lane] = true;
                } else if ( (check_object_s - car_s < 90) &&  (check_speed > car_speed) ) { //if object ahead has lower speed dont change into its lane
                    lane_taken[check_lane] = true;
                } else if ( ( car_s - check_object_s  > 30) //if car 30 meters behind me and has greater speed
                    && (check_speed > car_speed) )
                {
                    lane_taken[check_lane] = true;
                }

                if ( check_object_d< (2+4*lane+2) && check_object_d > (2+4*lane-2) ) { //check if object on same lane as our car
                    
                    if ( (check_object_s - car_s > 0) && (check_object_s - car_s < 30) ) //if (1) object is ahead of our car; AND (2) if the gap is smaller than 30meters
                    {
                        too_close = true;
                        front_car_speed = check_speed;
                    }
                }
            }

          // Control car speed and execute lane changing
          if (too_close)
          { //slow down
              //consider lane changing
              if ( (lane -1 >= 0) && (lane_taken[lane -1] == false) ) { //change left
                  lane = lane -1;
                  if (ref_vel < 49.5) // Speed up
                  {
                      ref_vel += 0.224;
                  }
              } else if ( (lane +1 <= 2) && (lane_taken[lane +1] == false) ) { //change right
                  lane = lane +1;
                  if (ref_vel < 49.5) // Speed up
                  {
                      ref_vel += 0.224;
                  }
              } else {
                  ref_vel -= 0.224;
              }
          }
          else if (ref_vel < 49.5) // Speed up
          {
              ref_vel += 0.224;
          }
            
            

          //1: create spline along the road curvature (in xy coord system) and sample points from it
          //1.1: For approx. trajectory begining (ie. spline start) == add two starting points related to car history (ie. previous path or if doesn't exist car position)
            
          //(previous path is returned by the simulator and accoutns for the path passed on by this trajectory planned in prior iteration, minus the waypoints that
          // were already crossed by the car as time passed from prior iteration)
            
          // If previous size is almost empty, use the car as a starting reference
          if (prev_size < 2)
          {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          } else //if more than two points in the previous path == use last two points and add it for charting new path
          {
              // Redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              // Use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }
            
          //1.2: Add on 3 extra points moving ahead from the current car position. (use Frenet car coords as starting point and add three points with even 30m spacing)
          // s = add in the 30m spacing
          // d = 4meters * lane_number + 2meters ( 2meters so the car is in the center of the lane)
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
            
          //1.3: additional "trick" - move from map coordinates to local car coordinates (this way inital spline position is zero, instead of some funky number changine in time)
          // (this makes it easier to samples points from the spline)
          for (int i = 0; i < ptsx.size(); i++)
          {
              // Shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
            
          //1.4: Create a spline that interpolates points calculated in 1.1 and 1.2
          tk::spline trajectory_spline;
          // Set (x,y) points to the spline
          trajectory_spline.set_points(ptsx, ptsy);
            
          //1.5: calculate the final points of the trajectory (take previous path with not yet "used up" points and add on points sampled from the spline)

          // Define the actual (x, y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //1.5.1: Start with all of the previous path points from last time
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //1.5.2: Add in points from the spline
          //First - Calculate how to break up spline into points so that we travel at our desired reference velocity
          double target_x = 30.0; //ending distance from the current position (meters)
          double target_y = trajectory_spline(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for(int i = 1; i < 50 - prev_size; i++)
          {
              double N = target_dist/(0.02*ref_vel/2.24); //2,24 == conversion from miles to km, 0.02 == time step used in similator for sampling points
              double x_point = x_add_on + target_x/N;
              double y_point = trajectory_spline(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Rotate back to normal map coordinates from local car coordinates after rotating it earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }
            
          json msgJson;

//
//            //APPROACH 1: move car in fixed distance steps along the road curvature
//            // Define the actual (x, y) points we will use for the planner
//            vector<double> next_x_vals;
//            vector<double> next_y_vals;
//
//            double dist_inc = 0.5;
//            for (int i = 0; i < 50; ++i) { //plan trajectory for 50 steps ahead
//                double next_s = car_s+(i+1)*dist_inc; //current car position + some fixed distance increment
//                double next_d = 6; //constant for middle lane, 0 +4 (4 = lane width) , + 2 = middle of the (middle/centre) lane
//                //convert to x,y coordinates as those need to passed to JSON
//                vector<double> xy = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
//                next_x_vals.push_back(xy[0]);
//                next_y_vals.push_back(xy[1]);
//            }

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
