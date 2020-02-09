# Highway Path Planning 


# Lane changes
<p align="center">
	<img src="/media/lane_change_showcase.gif" alt="result"
	title="result"  />
</p>

---

The goals of this project are the following:
- car is able to drive at least 4.32 miles without incident
- car obeys the speed limit
- car obeys safe acceleration and jerk limits
- car avoids collisions
- car stays in lane
- car can change lane if higher speed is possible

## 1. Approach

When I started this project I wanted to build an efficient and safe solution, which does not require heavy processing. To give my solution some sort of structure, I created a new file containing a Controller class, which does all of the above mentioned tasks. These tasks can be clustered in 2 categories:

1. Control Update:

The general task of the control update is to return a reference speed at which the car should travel and the optimal lane to drive on.

Let's start with the reference speed: in general we would like to travel at our set max velocity of ~23 m/s. This may change however if its not safe to drive at this speed due to a slower vehicle in front of us or on our target lane. Therefore we reduce the speed to the approximate speed that the spotted vehicle has. This is done by filtering the sensor fusion data for proximities with their s & d values compared to ours. 

The optimal lane to drive on follows an efficient approach: it's always optimal to drive on the road that the car is on if car's speed is equal to max velocity. This condition becomes false when we reduce speed because a slower vehicle is in front of us. Only then other lane options are evaluated. This evaluation features upfront identifing which lanes can actually be driven to, fed to a vector containing 1 or 2 values (2 if in middle). Afterwards first its checked if the lane change would be a safe option considering distances to other vehicles on that lane. Since the sensors are located at the front of the car, cars that are directly behind us still have a distance of 4-5m. Therefore I chose the option to mark a lane as safe when the distance of -35 - 20 is clear of other vehicles. Afterwards the lanes are weighted using 3 compontents: distance clear of vehicles at lane, possible velocity on lane and some custom weight that favors a change to middle due to having more lane options on the middle lane. When a lane change is initiated the current speed is kept untill we are at least on our target road to improve fitting in traffic and not colliding with the car in our old lane. Therefore I as well have some sort of State Machine with only 2 states: normal drive or changing lane.

2. Trajectory Update

Within the trajectory update I use the spline library to create a path for the vehicle to follow. The process flow is as follows:
- append not used trajectories from last update or get last & current position from position and steering
- create a spline from waypoints with distances of 50, 70 and 90 meters in Frenet
- with the given spline return the drivable distance within 0.02s with regards to acceleration and velocity

<p align="center">
	<img src="/media/speed_showcase.gif" alt="result"
	title="result"  />
</p>

The drivable distance takes as input our actual velocity, target velocity (when there is a delta this means car will accelerate or slow down) and the generated spline. With euclidean distance calculation the distances between waypoints of 0.02s are checked if the hurt the speed limit and are then decreased until they fulfill the requirements. This works not perfect - sometimes when changing lane in some curves pop up the max acceleration marker. I tried tweaking and improving but am not 100% sure if it fits since the reproducability of some very specific situations is not given, for example in the observation below, where a lane change is initiated on an already curvy path. After tweaking, I have not encountered this error again.

<p align="center">
	<img src="/media/max_acceleration_error.gif" alt="result"
	title="result"  />
</p>


## 2. Performance

I am quite satisfied with the performance of the path planning controller. Even though the implementation is a very naive approach that does not always choose the most optimal route, it is pretty consistent in handling also challenging situations. 

This includes all of the requirements of passing:

1. The car is able to drive at least 4.32 miles without incident..
When I tested, the car was easily able to go >10 miles without an incident.

2. The car drives according to speed limit..
The car tries to drive around 49 mph consistently.

3. Max acceleration and jerk are not exceeded..
Already talked about this, before tweaking it only happened in rare and hardly reproduceable situations.

4. Car does not have collisions
The car does not drive into other cars but may be tackled from behind if an approaching car has a high velocity. 

5. Car stays in lane ..
Yes.

6. Car can change lanes ..
Yes.


# Lane changes
<p align="center">
	<img src="/media/lane_change_showcase.gif" alt="result"
	title="result"  />
</p>


# Handling of blockades 
<p align="center">
	<img src="/media/blockades_showcase.gif" alt="result"
	title="result"  />
</p>


## 3. Reflection

Oh boy. This was actually a tough one, since the reproducability of some situations is actually so hard. My best-of featured cars on appending lanes with the same speed just randomly changing lanes and colliding with me, cars from behind just bumping me or just 3 cars blocking all lanes for 3 minutes and my car desperatly trying to find a solution to this dilemma. 

Since I went for a rather naive approach for a complex task, there is obviously still a huge room for improvement. Some of these are:

The sensor fusion module I wrote is somewhat lacklustered. First of all, cars in appending lanes are not tracked constantly, but only when considering a lane change. Therefore some cars that randomly change lane to our lane actually get identifed after they have passed half the distance. This is a major security lack that I would totally consider in a more complex environment. 


Additionally the lane change trajectory could as well be improved to better match the correct speed when changing lanes. Sometimes really fast cars can bump into you from behind. The identifier for a safe lane could therefore feature a comparison of delta velocity between my car and the approaching car and stop it in the tracks. On the other hand if the delta velocity is really low, you could also try to lower the safe distance to also fit into smaller gaps safely. 

<p align="center">
	<img src="/media/accident_showcase.gif" alt="result"
	title="result"  />
</p>


One other challenge in my implementation is actually the variety of parameters that have an influence on the driving. These are hard to optimize since each situation is really different and some ranges work better than others. To create some dependencies between observations to update these parameters while driving would also vastly improve the situational awareness. 







