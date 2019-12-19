#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
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
  
  // Define lane to stat with (0, 1 or 2) left most, Middle and right most lane
  int lane = 1;
  
  // Define a reference velocity intended for targeting. In our case we don’t want to exceed 50 mile/hr
  double ref_vel = 0.0;

  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          
          // Helps with transitions between paths 
          int prev_size = previous_path_x.size();

          json msgJson;

          // Check if there is a car in front of us, if yes, how close is it to us
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
         
          /******************************************************************************
          
          Prediction 
          
          *******************************************************************************/
          
          bool agent_right = false, agent_left = false, agent_ahead = false;
          int count_left = 0, count_right = 0;
          
          // Loop through all available agent cars
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // Check in which lane the agent car is 
            float d = sensor_fusion[i][6]; // returns the d value (fernet coordinate) of the corresponding agent car
            int agent_lane = -1;
            
            if (d > 1 && d < 3) // left lane (center = 2)
            {
              agent_lane = 0;
            }
            else if ( d > 5 && d < 7) // middle lane (center = 6)
            {
              agent_lane = 1;
            }
            else if ( d > 9 && d < 11) // right lane (center = 10)
            {
              agent_lane = 2;
            }
            else // the car is not at any of the three lanes
            {
              continue; 
            }
            
              
            // Find the agent car's velocity to help predict its position in the future
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(pow(vx,2) + pow(vy,2));
            double check_car_s = sensor_fusion[i][5]; // returns the s value (fernet coordinate) of the corresponding agent car
                
            // Looking at where the agent car will be in the future
            check_car_s += ((double) prev_size * 0.02 * check_speed);
   
            // Check if the agent car is ahead of us, on our right or our left
            double difference  = car_s - check_car_s;
            double min_gap = 30; // minimun gap allowed between the ego car and agent cars (safety distance)
            
            // An agent car is infront of the ego car and violated the safety distance
            if (agent_lane == lane && (check_car_s > car_s && difference > -min_gap)) 
            {
              agent_ahead = true;
            }
            // An agent car is on the left lane within a distance less than the min_gap ahead or behind the ago car
            else if (agent_lane - lane == -1 && difference > -min_gap && difference < min_gap) 
            {
              agent_left = true;
            }
            // An agent car is on the right lane within a distance less than the min_gap ahead or behind the ago car
            else if (agent_lane - lane == 1 && difference > -min_gap && difference < min_gap) 
            {
              agent_right = true;
            }
            // Look a little bit ahead +/- 60m from the min_gap. This will help in taking a better decision if both the left and right lanes are free 
            else if (agent_lane - lane == -1 && difference > -min_gap-60 && difference < min_gap+60)  
            {
              count_left++;
            }
            // Look a little bit ahead +/- 60m from the min_gap. This will help in taking a better decision if both the left and right lanes are free  
            else if (agent_lane - lane == 1 && difference > -min_gap-60 && difference < min_gap+60)  
            {
              count_right++;
            }
          }
          
          /******************************************************************************
          
          Behavior Planning
          
          *******************************************************************************/
          
          
          // Take the best decision
          double speed_increment = 0.0; // mph
          
          // Cases when there is an agent car in front of us
          if (agent_ahead)
          {      
            
            // If the ago car is in the middle lane and both the left and right lanes are free
            if (!agent_right && !agent_left && lane == 1)
            {
              // If in the near future I have less agent cars on the left lane, the ago car will change to the left lane
              if (count_right >= count_left)
              {
                lane -= 1;
              }
              else
              {
                lane += 1;
              }
            }
            
            // If the ago car is not on the left most lane and the lane to its left is free, move to the left lane
            else if(!agent_left && lane != 0)
            {
              lane -= 1;
            }
            
            // If the ago car is not on the right most lane and the lane to its right is free, move to the right lane
            else if (!agent_right && lane != 2)
            {
              lane += 1;
            }
            
            // Changing lanes is not possible, just reduce the speed 
            else
            {
              speed_increment -= 0.224; // 5m/s
            }
            
          }
          
          // If no agent car is in front of us
            else
            {
              // If the ago car is not in the middel lane, it will move to the middle lane if possible
 
              if(lane == 0 && !agent_right)
              {
                lane += 1;
              }
              else if (lane == 2 && !agent_left)
              {
                lane -= 1;
              }
              
              // If the ago car is in the middel lane and its velocity is less than the maximum speed, increase it!
              if (ref_vel < 49.5)
              {
              speed_increment += 0.224;
              }
            }
              
          
          /******************************************************************************
          
          Trajectory Generation
          
          *******************************************************************************/
         
          
          // Generate sparsed waypoints (ex. 30m apart) to incorporate them with a spline and fill it with more points that will help in controlling the speed of the vehicle.  
          vector <double> ptsx;
          vector <double> ptsy;
          
          // Keep track of the reference point - x,y and yaw - (either the starting point of where the vehicle is or the last point of the previous path)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // Check the previous path size, it might be almost empty in which we will use the car as the starting reference or has some points left that can be made use of.
          
          // If the previous path is almost empty, then from the car, I would like to create a starting reference of the new path that is tangent to the car's angle (generate two points to make sure the path is tangent)
          
          if (prev_size < 2)
          {
            // Create a path that is tangent to the car's angle 
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            // First point
            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);
            
            // Second point
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          }
          // If we still have points left in the previous path, we will use the endpoint of the previous path to be the reference 
          else
          {
            // Set the reference variables to be equal to the endpoint of the previous path
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            // Find the point prior to the one set as the reference so it can be used as to make the new path tangent to the endpoint of the previous one 
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // First point
            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);
            
            // Second point
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
            
          }
          
          // Define three other points in fernet equally spaced by 30m starting after the referance point  
          // d = 6 = 2 (distance from yellow line to mid of left lane)+ 4 (distance from center of the mid of the left lane to the mid of middle lane) * (1) (Current lane)  = 6
          
          vector <double> next_w;
          
          for (int i = 30; i <= 90; i+=30)
          {
            next_w = getXY(car_s + i, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              
              ptsx.push_back(next_w[0]);
              ptsy.push_back(next_w[1]);
          }

          
          // Transfer to the local car's coordinates for the ease of calculations (transformation and rotation to have (x, y, yaw) = (0, 0, 0))) 
          for (int i = 0; i < ptsx.size(); i++)
          {
            
            // Shift the car's refrence angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
            
            // Smooth the path by using the spline method instead of Polynomial fit. The advantage of using spline is that it is guaranteed to pass through all the points      			   (Piecewise function of polynomials)
            // Define a spline
            tk::spline s;
            
            // Set the set of x and y points we generated earlier to the spline
            s.set_points(ptsx,ptsy);
            
            // Define sets that will include the points used for the planner (path planning points)
            vector <double> next_x_vals;
            vector <double> next_y_vals;
            
            // If I have some left points from the previous path (the simulator didn’t go through them) - Helps in getting a smooth transition 
            for (int i = 0; i < previous_path_x.size(); i++ )
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            

            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
            double x_add_on = 0;
            
            // Feed more planning path points - Filling with the number of points needed to reach 50 after adding the left points from the previous path
            for (int i = 0; i <= 50 - previous_path_x.size(); i++)
            {
              
              // Make sure we don't exceed the maximum speed and acceleration 
              ref_vel += speed_increment;
              
              // If the reference velocity is greater than the maximum velocity, set the reference velocity to the maximum velocity
              if ( ref_vel > 49.5 )
              {
                ref_vel = 49.5;
              } 
              // If the reference velocity is less than the minimum acceleration, set the reference velocity to the  minimum acceleration
              else if ( ref_vel < 0.224 )
              {
                ref_vel = 0.224;
              }
              
              double N = (target_dist / (0.02 * ref_vel / 2.24));
              double x_point = x_add_on + (target_x) / N;
              double y_point = s(x_point);
              
              // Update x_add_on for the next iteration
              x_add_on = x_point;
              
              // Variables used for shifting back to the global coordinates from the local coordinates.
              double x_ref = x_point;
              double y_ref = y_point;
              
              // Rotating back
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
              
              // Transitioning back
              x_point += ref_x;
              y_point += ref_y;
              
              // Add to the planning path points sets
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
           

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