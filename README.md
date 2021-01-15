# **Path Planning** 

## Udacity Project 7 Writeup

---

**Path Planning Project**

The goals / steps of this project are the following:
* drive a car in the virtual highway implemented in the simulator
* car must not hit other vehicles, not exceed speed limit (50 MHP) but not drive too slow either
* car must not exceed total accelation 10m/s^2 and not excced jerk of 10 m/s^3
* car should be capable of changing lanes when the traffic ahead is too slow and when other lanes are empty
* car should be able to drive min. 4.32 miles without incident.
* car doesn't spend more than a 3 second length out side the lanes and stayed on the allowed lanes only

---

In comparison to course material the key code modificatoons / customizations are in "main.cpp" and related to:
* driving car with proper speed, without crashing into other cars, without exceeding allowed accelation and jerk - I implemtned this following pretty much exectly the Udacity Q&A video tutorial for Project 7
* lane changing - implemented based on monitoring state of all lanes (rather than FSM for own car)

[//]: # (Image References)

[image_1]: ./img/1hour_simulator_drive.png "Simulator testing"

**Summary:** For this project I followed the extensive project Q&A video, which actually helped to implement everything in a simplistic and minimum required version for the project except of lane changing. I picked up where the Q&A left and implemented two code blocks: one for logic related to analyzing traffic for each lane and second block for decision making related to lane change. See details on each below. I found the main challenges were related to tuning the exact lane changing decision boundaries for the car (ie. akin to cost function parameters) and the randomness of the simulator making it difficult to repeat edge cases that caused failures in driving. **Testing wise, for final solution I left the car driving in simulator for 1 hour (\~43 miles) , managing to go without any incident until then and beyond.**

![alt text][image_1]

---

## Implementation Details

### Driving car on its own lane

I implemented this part following the instructions from the project Q&A. My code is pretty much exactly the same as the one from Udacity. I just made some small re-arrangements and comments in a way that made more sense to me. The code responsible for determining trajectory that follows the road curvature, driving at max. allowable speed on own lane and avoiding collision is located in lines 205-314.

### Lane Changing 

This is the only real addition on top of what was provided. Few lines of code but actually a lot of testing and frustration. Instead of doing a Finite-state machine for the car and a cost function, I thought it would be simpler to constantly monitor the state of all three lanes and update if they are taken or not. 

The code for monitoring state of lanes is located in lines 143 -165, inside the sensor_fusion loop (as implemented part of the Q&A). Firstly, I assign every object to a lane (using equation similar to one from Q&A for determining if object is on car's lane); later based on several conditions I determine if lane is taken or not: 
* I've taken a quite conservative approach for lane changing requiring plenty of space (ie. 30 meters ahead and 20 behind)
* another rare case that I tried to mitigate was all three lanes occupied with cars, on rare occasions that caused the vehicle to bounce from one lane to another. Here apart of presence or absence of object on lane, I check for its speed (and don't switch to lanes that are equally blocked)
* additionally, similar as checking for object on lane and their speed ahead of the vehicle, I also do the same behind and don't allow lane change if there is a faster moving car behind

The code for executing lane change is located in lines 181 -195. This part is a modified version of the speed adjustment section from Q&A - if the road ahead is taken by a slow vehicle apart of slowing down I additionally check if there is an option to change lanes. This is quite basic, except of one modification where apart of lane changing I also make sure that the car is not slowing down and accelerate when doing the lane change.

## Challenges

* simple bugs cause very expected behavior - my first issue was that at some seemingly random moments car tended to stop driving or behaved against the coded rules. Turned out that at very very rare occasions the sensor_fusion data contains objects that do not belong to any lane - originally I didn't account for that as a result of which my code was writing into inexistant position of a vector, probably messing up some other variable and creating very erratic behavior.

* edge cases - the simulator generating traffic situation on random makes it difficult to re-create some rare situations that cause issues (crashes etc.). Sometimes I had to wait for half an hour to see the same situation re-appear again. This relates also to another problem of tracking what happened leading to a crash or some unwanted behavior. For debugging I ended up doing video screen capture with simulator window next to log window - having both in sync and to an extent capable to connecting the road situation with all data in path_planning code.

* simulator performance - similar to previous Project, I find the simulator making the task more difficult rather than easy. Having the 3d view is nice but I think very unnecessary. Ideally there should be simplified 2d top view, without too much graphics. Currently, the simulator takes a lot of machine resources and (in my configuration) after running for a while, e.g. 10-20min sometimes the communication path_planner <-> simulator seems to fail. I didn't fully track the origin of the issue but it appears like some data is missing (either not passed to path_planner code or packets lost). 

## Possible improvements

* The current speed adjustment is very basic. The most annoying part is when there is a slow moving car ahead and no option to change lane, our vehicle is slowing down and speeding up like a yo-yo. I spent some time trying to mitigate this but eventually I didn't manage to get it working properly therefore did not include in the solution.

* Finite-state machine for vehicle and explicit cost function. My solution of tracking lane states and implicitly weighting cost of lane changing via various conditions is quite simplistic, I thinking to try to the FSM but as I was running out of time until deadline, so I decided to skip it. Adjusting the cost function would undoubtably take more time).

--- 

## Project Execution Info (by Udacity) 

### Goals

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car's localization and sensor fusion data are provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```


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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

