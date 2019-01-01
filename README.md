# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
This project uses the Udacity Term3 Simulator which contains the Path Planning Project, available here from the releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car goes as close as possible to the 50 MPH speed limit, avoids hitting other cars, avoids acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3, and passes slower cars.


## Details

### Trajectory generation

**Creating a smooth spline**
The basic idea in trajectory generation is to increment the Frenet 's' value to move the car forward and then transform to x/y coordinates. However, this simple method does not produce smooth trajectories, as the path abruptly shifts at each map waypoint. Instead, a spline is created using a line tangent to the car's previous path end position (lines 501-525) and three points evenly spaced in Frenet 's' at 30, 60, and 90m ahead (lines 529-539). This ensures that the car is following the desired curvature of the road with no abrupt change in heading.

**Appending previous path**
The path length sent to the simulator is always 50 elements long. The first part of the path copies whatever of the previous path has not yet been achieved (lines 561-563).

**Using shifted perspective to enforce smooth velocity**
The rest of the path is filled with evenly interpolated points. The X value is incremented by the ratio of the distance in x to the total distance multipled by the reference velocity * dt. The ensures that x is incrementing at the right velocity. From this x value, the y value is calculated using the previously generated spline (lines 566-575). The x and y value are shifted to the car's perspective and back to make the math easier, then added on to the path (lines 541-548 and 583-590).

### Adjusting speed

Generally, the car attempts to drive close to as fast as possible at 49.5mph. However, the set velocity adjusts depending on the car's speed and the speed of the surrounding cars. If the car is not moving and there are no obstructions, the set velocity will slowly increase from 0 to 49.5mph. If a slow car is ahead, the set velocity will slowly decrease. If a slow car is suddenly ahead, the car will attempt to decrease its velocity faster - this is a choice between exceeding acceleration limits and not crashing. This velocity change is implemented in lines 483-490.

### Detecting slower cars

At each iteration, the car uses data from sensor fusion to check information on the other cars. The 's' value of the car's path end is compared to the expected 's' value of the other cars using a constant velocity and time to path end. If a car is in the same lane and closer than 30m, a check is set that the car is too close. If a car is in the same lane and closer than 10m (which can happen if a slower car suddenly shift into the lane), a check is set that the car is far too close (lines 420-420). In either case, the car will start to decrease its speed and look for an available lane change.
The other lanes are also checked for cars to see if they are available given the desire to change lanes. If there is a car less than 20m behind or less than 20m ahead, or even less than 40m behind if it is going more than 5mph faster than our car, the lane is set to occupied (lines 430-443).


### Changing lanes
If a slower car has been detected ahead, the car will attempt to switch lanes to the left. If the left lane is occupied, the car will attempt to switch lanes to the right. During the lane change, the car cannot change its desired lane. The lane change is considered done when the car's 'd' position has shifted by more than 3.3m. If either shift is impossible because of surrounding cars or because it would entail the car driving off the road, the car waits for an opportunity to change. This logic is implemented in lines 446-481.

## Previous attempts

Most of the code is borrowed from Aaron Brown's demonstration in the project FAQ. It also uses the Bezier curve fitting code in spline.h suggested there. I first tried this with few additional resources, adding methods for jerk minimizing trajectory generation, generating splines based on least squared error for a polynomial fit, and using the spline.h code to generate splines based on frenet distances. 

Jerk minimizing trajectory generation didn't work very well because there was no intuitive way for me to limit the acceleration and jerk, and many of the produced trajectories violated the acceptable limits. I also had a hard time coming up with proper distances and times to use. 

I moved from this to incrementing based on the Frenet 's' distance, and ran into the problem of smooth transitions at waypoints. I tried using the polynomial curve fitting method I implemented, but then had problems switching abruptly from one smooth curve to another smooth curve. I played around with adding more points in the spline calculation, starting the curves in between waypoints, and using previous path information. My ideas did not work out very well for this smoothing, and I turned to the project FAQ. 

Aaron's use of the previous path last two points to force a smooth tangent line solved the problems I had been having with the transition, and his use of shifted the perspective and planning in the x and y space solved the problem I had been having with fluctuating velocities as I appended points to the previous path. I had not yet attempted to use sensor fusion information anywhere, and Aaron's base code worked pretty well for me. I added in checks to make sure a lane change wouldn't cause a crash, and a way for the car to shift lanes in both directions.

The code could be improved by a cost function to check which lane is the most desired based on the near car velocities, but it also seems to work rather well as is.