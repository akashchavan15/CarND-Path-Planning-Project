In the following subsections, I will first describe the path generation method in more detail and 
then address how I managed to pass each rubric point in order to avoid any "incidents",
as classified by the simulator, over the 4.32 mile long track.

The path generation model follows the project walk through by udacity 
(https://classroom.udacity.com/nanodegrees/nd013/parts/30260907-68c1-4f24-b793-89c0c2a0ad32/modules/b74b8e43-47d1-47d6-a4cf-4d64ea3e0b80/lessons/407a2efa-3383-480f-9266-5981440b09b3/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d)

All cars from sensor fusion data are examined and I am performing a high level decision whether to change lanes or speed.
Follwing is the overall summary of decisions which can be taken by host car,

1. Is there a car in front of host car so that the host car should change lane or slow down 
2. Are there cars obstructing the right or left lanes so we cannot change lanes and must slow down? 
   This also includes cars that are switching lanes and thus have a horizontal velocity component in Frenet space. 
3. Also, we do not change lanes if we are already in a right-most or left-most lane; we also don't perform a 
   double-lane-change or a lange change below 20mph.   
4. Is the distance to the closest car in front of us in the target lane greater than the distance to the car in
   front of us in our lane
   
A spline is constructed to sample new trajectory points from 

1. Its first control points are the last two points of the non-processed/remaining path that is returned by the simulator.
2. If currently no points were returned by the simulator, it starts with the current vehicle position.
3. To that, 3 more control points with distance 30, 60 and 90m ahead of the last point in Frenet space are added. 
4. The spline control points are transformed to vehicle coordinates with x pointing ahead, so that the spline is a
   well-defined function with unique x-to-y resolution for all x. 
   
The spline is sampled to return the trajectory points that the path should follow

1. The spline is first filled with the points that were returned from the simulator and were not processed yet 
2. The spline is always sampled up to 30m ahead 
3. The number of points that are sampled (and the distance between the sampled points) is determined by the reference velocity, 
   so that the car covers the correct distance between each trajectory point of which one is passed every 0.2s.
   
In the following subsections I go into detail about the details in the code that make me cover each aspect of the 
rubric requirement of "no incident occuring".

Driving as Per Speed Limit:

1. The speed limit is checked first in (see above) by only increasing the reference
   velocity if the speed limit is not reached
   
   else if (host_car_ref_vel < 49.5) {
	// if we are too slow, speed up (following max. accell. & velocity)
	host_car_ref_vel += .224;
   }
   
2. In order to follow the reference velocity, the distance and number of data points sampled along the spline in 
   is dependent on the reference velocity 
  
  // calculate spacing of points needed to find reference velocity
  double num_points = target_dist / (.02*host_car_ref_vel/2.24);
  
Driving as Per Maximum Acceleration and Jerk Limit:

1. Maximum acceleration and jerk are followed by only increasing and decreasing the velocity in small
   increments 
   host_car_ref_vel -= .224;
   host_car_ref_vel += .224;
   
2. Even then, some exceeding jerk and acceleration can happen if a lane change across multiple lanes occured. 
   In order to register whether we are changing lanes, I compare the current lane, derived from 
   the car's d-value (0..4m for lane 0, 4..8m for lane 1 etc.)
   short host_car_lane = ((short)floor(host_car_d/4));
   
Avoiding Collisions:

This was by far the most challenging requirement and is currently probably implemented rather conservatively, 
leading to some slow-downs where speeding up and snugly fitting into a gap could probably improve performance. 
It was mostly handled by the policy implented above

I change_lanes_or_slow_down (== true) if any car is currently ahead of host car. But in order to make the final 
decision which of the two options it is, I also account for the following high-level observations:

1. I check if the right lane or left lane is free of traffic (dont_go_right, dont_go_left)
2. I also do not double-change lanes (changing_lane, see also previous section)
3. I do not change lanes if host car's speed is below 20mph.

To derive the variables change_lanes_or_slow_down, dont_go_left and dont_go_right, from the low-level sensor fusion data of each 
car on the road, more and more high-level features are generated such as longitudinal/lateral distances, velocities.

Longitudinal: First I infer if the car is closer than 30m ahead , closer than 15m behind or just in an 
              "inner circle" of 10m ahead or behind.

Lateral: This follows a 3-layered approach:

1. Additional velocity metrics are calculated by trigonometry , out of this "falls" 
   also a feature that tells us, if the car is slower than host car.
2. The car's lane is calculated from its d value and a potential merge / lane-change of a car to another lane
   is inferred by a significant vd value.
3. Based on other_car_lane and other_car_merging_lane, the high-level flags that specify if a car
   is on host car's lane or the lanes next to host car are set (other_car_is_in_my_lane, other_car_is_left, other_car_is_right).
   
Combining both to determine if the car is an obstacle for lane change. 
		bool other_car_is_obstacle = ( (other_car_is_ahead && other_car_is_slower)
								  || (other_car_is_behind && !other_car_is_slower)
								  || other_car_is_close
						  );
		// don't switch lanes if the car is dangerous
		dont_go_left  = dont_go_left || (other_car_is_left && other_car_is_obstacle);
		dont_go_right = dont_go_right || (other_car_is_right && other_car_is_obstacle);

As indicated initially, I admit that this approach is far from perfect. Especially the heuristic, 
that tries to guess whether a car is changing lanes sometimes fails. It seems that other_car_vd, 
which is calculated relying on the normal vector of the closest waypoint, is inaccurate, 
leading sometimes (especially in curves) to predictions of incorrect lane changes of other cars. 
A resolution would probably be to find a more accurate normal vector by sampling between two waypoints or 
by finding a better heuristic, potentially involving tracking of cars over various simulator cycles.

Staying Inside Reference Lane:
Staying inside the reference lane is achieved by keeping d constant in the interpolation of the target trajectory

	vector<double> controlpoint = helper.getXY(host_car_s+spacing, (2+4*host_car_ref_lane), 
			map_waypoints_s, map_waypoints_x, map_waypoints_y);
			
Smoothly Changing Reference Lanes:
Changing lanes smoothly is achieved by modifying host_car_ref_lane. Since always some portion of the previous path is 
kept,the spline interpolation between the old trajectory segment and the new segment,
now with updated host_car_ref_lane, results in a smooth trajectory.
In addition to the safety checks, lane changes are only performed when they make sense. This includes checking via 
three minimum-variables which car is the closest car ahead in the current lane as well as in the lane to the left
and to the right 

	// don't switch lanes if leading car in target lane is closer than
	// leading car in current lane
	dont_go_left  = dont_go_left  || (min_dist_left < min_dist_here);
	dont_go_right = dont_go_right || (min_dist_right < min_dist_here);