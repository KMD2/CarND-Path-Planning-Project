# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Reflection 

The code model for generating paths contains three components; prediction, trajectory planning, and trajectory generation as described below:

### 1) Prediction

For prediction, I used the concept of Finite State Machine (FSM) in which I have states and transitions. The code corresponding to prediction is between lines 112 and 185.
With the aid of the sensor fusion data, I start by looping through all available agent cars in the vicinity of my ago car. By the information given about the d coordinate, I find in which lane exactly is the agent car. After that, by calculating the speed of the agent car, I predict its position in the future (s value). I assumed a safety gap of +/- 30m between the ego car and any other agent car. By the information of the agent car's lane and the minimum allowed gap, I create three boolean states; agent_ahead, agent_left, and agent_right. I have also added two cases in which they tend to sneak a little bit to the near future, in order to break the tie efficiently in a situation where the ego car is in the middle lane and both the right and left lanes are free. In that case, I keep a record of the number of agent cars that are on the left and right of my ego car within a distance of +/- 90m.

### 2) Behavior Planning 

In this part (lines 186-262 in the code), depending on the prediction step, the ego car will choose the most suitable action to perform (transitions). If an agent car was detected to be in front of the ego car, there are three options; if the ego car is in the middle lane and both the right and left lanes are free, the ego car will move to the lane with fewer agent cars (within +/- 90m from it). Otherwise, it will move to the right lane if the left lane is not free and the ego car is not in the rightmost lane or it will move to the left lane if the right lane is not free and the ego car is not in the leftmost lane. If changing lanes is not an option, then I just reduce the speed to avoid collision.

If there is no agent car ahead, I assume that the ideal lane for the ego car is for it to be in the middle lane. So, I check if it is not in the middle lane, the car will take a suitable action to move back there. Otherwise, if it is already in the middle lane, I check if the reference vehicle is less than the maximum speed, then I increase it.

### 3) Trajectory Generation

*Mostly based on the project Q&A video* 

For the trajectory generation, I used the spline method in favor of the polynomial fit because it results in a smoother path (it is guaranteed to pass through all the points -piecewise function of polynomials -). To generate a spline, I generated a set of 5 points. The first two are the points that will create a tangent path to the ego car's angle from the reference point. The reference point is either the endpoint of the previous generated path or otherwise, the point of the car's position. The other three points are generated in fernet, equally spaced by 30m starting after the reference point. 

Next, I generate the planning path points. If I have some points left from the previous path, I reuse them and add more points until I reach a total of 50 points. Reusing the remaining points from the previous path helps in providing smoother transitions. Moreover, while generating the points, I make sure that the reference velocity and acceleration don't exceed the allowed limits 

