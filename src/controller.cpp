#include <vector>
#include <iostream>
#include "spline.h"

using std::vector;


class Controller{
public:

  // update module
  void update_control(vector <vector<double>> sensor_fusion, double car_s, int prev_size){
    
    vector <int> car_lanes;
    vector <double> car_pos;
    vector <double> car_speed;
    vector <int> car_id;

    // make vectos for cars
    for (auto other_car: sensor_fusion){
      // if car is on other side of road its not relevant
      if (other_car[6]>0.0){
        car_id.push_back(other_car[0]);
        car_lanes.push_back((other_car[6]/4)+1);
        car_speed.push_back((sqrt(other_car[3]*other_car[3] + other_car[4]*other_car[4])*0.44704*2)/(prev_size*0.02)); // 
        car_pos.push_back((other_car[5] + (car_speed[car_speed.size()-1])*0.02*prev_size)-car_s);
      }
    }

    std::cout << "Lane: " << lane << std::endl;
    double new_v = max_v;

    // do we need to decrease our speed?

    for (int i = 0; i<car_lanes.size();++i){
      if (car_lanes[i] == lane) {
      	if (car_pos[i] > 0.0){
      		std::cout << "Car in lane  " << car_pos[i] << "  " << car_speed[i] << std::endl;
      	} 
        if (car_pos[i]< 40.0 && car_pos[i] > 0.0) {
          if (new_v > car_speed[i]){
            new_v = car_speed[i];
          }
        }
      }
    }

    if (lane_change) {
    	target_v = target_v;
    }
    else {
    	target_v = new_v;
    }

    

    // if we decreased our speed, is lane change favorable?
    if (target_v < max_v && !lane_change){

      // we define the lane options that we have to check them for safety
      vector <int> lane_options = possible_lanes();
      vector <int> safe_lanes;

      // check if any vehicles in close proximity
      for (auto opt: lane_options){
        bool is_safe = true;
        for (int i=0;i<car_lanes.size();++i){
          if (car_lanes[i] == opt) {
            if(car_pos[i] - car_s > -35.0 || car_pos[i] - car_s < 35.0){
              is_safe = false;
            }
          }
        }
        safe_lanes.push_back(opt);
      }

      // now we have filtered out safe lanes or are not left with any
      // if size == 0, we dont need to calculate possible changes since there is no safe option
      // if there is 1 or 2 options we need to calculate weights for each option and compare them with current weight
      if (safe_lanes.size()>0){
        vector <double> weights;
        vector <double> safe_speed;
        for (auto l: safe_lanes){
          // 3 weights, distance ahead clear, possible velocity, lane weight
          double min_dist = 250;
          double pos_vel = max_v;
          for (int i=0;i<car_lanes.size();++i){
            if (car_lanes[i] == l){
              if(car_pos[i] > 0.0 && car_pos[i]<min_dist){
                min_dist = car_pos[i];
                pos_vel = car_speed[i];
              }
            }
          }
          double w = get_lane_weight(l, min_dist, pos_vel);
          weights.push_back(w);
          safe_speed.push_back(pos_vel);
        }

        // compare weight to current 
        int best_lane_id = 0;
        if (safe_lanes.size() == 1){
        	best_lane_id = 0;
        }
        else {
        	if (weights[0]>weights[1]){
        		best_lane_id = 0;
        	}
        	else {
        		best_lane_id = 1;
        	}
        }
        double w_current = get_lane_weight(lane, 40, target_v);

        // set new lane if better weight
        if (weights[best_lane_id] > w_current){
          lane = safe_lanes[best_lane_id];
          target_v = safe_speed[best_lane_id] * 0.75;
          lane_change = true;
          lane_change_c = (45 / target_v);
          lane_change_c *= 15;
        }
      }
    }
  }


  // trajectory module
  vector <double> get_drivable_distance(int n_points, tk::spline s){

    	double max_acceleration = 0.045;
		double x_ref, y_ref;
		double a,b,c, max_d;
		int loopbreak = 0;
		vector <double> x_pts;


	    for (int i = 0;i<n_points;++i){

		    if (actual_v != target_v){

			   	if (actual_v < target_v && actual_v+max_acceleration < target_v){
			    	actual_v += max_acceleration;
			    }
			    else if (actual_v > target_v && actual_v-max_acceleration > target_v){
				    actual_v -= max_acceleration;
			    }
			    else {
				    actual_v = target_v;
			    }
			}

			max_d = 0.02 * actual_v;
			a = 0.02 * actual_v;
			b = s(x_ref+a-y_ref);
			c = sqrt(a*a+b*b);

			while (c >= max_d && loopbreak < 1000){
				a -= 0.05;
				b = s(x_ref+a-y_ref);
				c = c = sqrt(a*a+b*b);
				loopbreak++;
			}

			x_ref += a;
			y_ref += b;
			x_pts.push_back(x_ref);
		}
		return x_pts;
  }

  void update_trajectory(
    double car_x, double car_y, double car_yaw, double car_s,
    vector <double> previous_path_x, vector <double> previous_path_y,
    vector <double> map_waypoints_s, vector <double> map_waypoints_x, vector <double> map_waypoints_y){

    // clear last trajectories
    x_trajectory.clear();
    y_trajectory.clear();

    int prev_size = previous_path_x.size();
    if (lane_change){
    	std::cout << "Lane change ongoing" << std::endl;
    }
    else {
    	std::cout << "Normal Driving" << std::endl;
    }
    //std::cout << lane_change_c << std::endl;


    if (lane_change){
    	lane_change_c -= (50-prev_size);
    	if (lane_change_c <= 0){
    		lane_change = false;
    	}
    }

    // if we still have observations will append 
    for (int i = 0;i<prev_size;++i){
      x_trajectory.push_back(previous_path_x[i]);
      y_trajectory.push_back(previous_path_y[i]);
    }

    vector <double> t_x;
    vector <double> t_y;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    if (prev_size < 2){
      t_x.push_back((car_x - cos(car_yaw)));
      t_x.push_back(car_x);
      t_y.push_back((car_y - sin(car_yaw)));
      t_y.push_back(car_y);
    }
    else {
      ref_x = previous_path_x[prev_size-1];
      ref_y = previous_path_y[prev_size-1];

      double ref_x_prev = previous_path_x[prev_size-2];
      double ref_y_prev = previous_path_y[prev_size-2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      t_x.push_back(ref_x_prev);
      t_x.push_back(ref_x);
      t_y.push_back(ref_y_prev);
      t_y.push_back(ref_y);
    }

    for (auto wp: {50,70,100}) {
      vector <double> new_wp = getXY(car_s + wp,(-2 + lane * 4), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      t_x.push_back(new_wp[0]);
      t_y.push_back(new_wp[1]);
    }

    for (int i = 0; i<t_x.size();++i){
      double shift_x = t_x[i] - ref_x;
      double shift_y = t_y[i] - ref_y;
      t_x[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
      t_y[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
    }

    tk::spline s;
    s.set_points(t_x,t_y);
    int n_points = 50 - prev_size;

    for (auto x: get_drivable_distance(n_points, s)) {
      double y = s(x);
      double x_p,y_p;
      x_p = (x*cos(ref_yaw)-y*sin(ref_yaw));
      y_p = (x*sin(ref_yaw)+y*cos(ref_yaw));
      x_trajectory.push_back(ref_x+x_p);
      y_trajectory.push_back(ref_y+y_p);
    }
  }

  vector <double> get_x_trajectory(){
    return x_trajectory;
  }
  vector <double> get_y_trajectory(){
    return y_trajectory;
  }

private:
  // class vars
  int lane = 2;
  vector <double> x_trajectory;
  vector <double> y_trajectory;

  int stay_at_speed = 0;

 
  // fixed params
  double actual_v = 0.0;
  double target_v = 24.4;
  double max_v = 24.4;

  bool lane_change = false;
  int lane_change_c = 0;

  vector <double> lane_weight = {0.8, 1.0, 0.8};

  // helpers
  vector <int> possible_lanes(){
      vector <int> moves;
      if (lane == 1){
        moves.push_back(2);
      }
      if (lane == 2){
        moves.push_back(1);
        moves.push_back(3);
      }
      if (lane == 3){
        moves.push_back(2);
      }
      return moves;
    }

    double get_lane_weight(int l, double vehicle_distance, double possible_velocity){
        double w1 = vehicle_distance / 150.0;
        double w2 = possible_velocity / max_v;
        double w3 = lane_weight[l-1];
        return (w1*w2*w3)/3;
    }
};