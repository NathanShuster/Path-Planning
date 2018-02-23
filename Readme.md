Path Generation:


The key to this project is using a spline to model a viable path. A walkthrough of the method is detailed below.


To save computational power, this model frequently reuses the previous iterations projections. However, in the first iteration and further instances where data is scarce, we have to create our own. Thus the path generation algorithm begins by finding the point where it would’ve just been given it’s current orientation and angle, and uses that as a reference point.


            // without ref data, get points directly behind us to orient us
            if (prev_size < 2) {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);


              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);


            }


Afterward, it can use the previous model’s last point as the starting reference.


            // use previous path's end point as starting reference
            else {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];


              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);


              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);


            }


It then finds points 30m, 60m, and 90m in front of the car in the desired lane.
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);


            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);


            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);




Does a quick shift to local car orientation to make calculations easier. 


            for (int i = 0; i < ptsx.size(); i++) { //change from global to car orientation
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;


              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));


            }


Brings in the spline module and fits the orientation point and the three future (30, 60, 90m) points.
            tk::spline s;


            // fit benchmarks to spline
            s.set_points(ptsx, ptsy);


Adds the leftover points from last iteration to next_x_vals, next_y_vals.


                  vector<double> next_x_vals;
                  vector<double> next_y_vals;


            // get the points from the past iteration
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }


Creates a target point 30m away on the spline and fills out the rest of next_x and next_y vals using it.


            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));


            double x_add_on = 0;


            // fill it in with new points drawn along the spline
            for (int i = 1; i < 50 - previous_path_x.size(); i++)
            {
              double N = (target_dist/(0.02 * ref_vel/2.24));
              double x_point = x_add_on + target_x / N;
              double y_point = s(x_point);


              x_add_on = x_point;


              double x_ref = x_point;
              double y_ref = y_point;


              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));


              x_point += ref_x;
              y_point += ref_y;


              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);


            }


All done!


            json msgJson;
            msgJson["next_x"] = next_x_vals;
                  msgJson["next_y"] = next_y_vals;








Lane shifting logic:


Won’t go into as much detail here. Basically, if a car is in front of me forcing me to slow down, I first check if the left lane is open. If it’s clear, then I go left. Else, I check if the right lane is open. If that’s clear I go there. If both are closed, then I use another function to slow down and “trail” the car in front of me.


Trailing/following:


I have a bunch of lines of code that culminate in the following line.


velTarget = car_speed + (0.02) * (distBetweenCars - targetFollowingDist - velDiff * 3); 


I have the targetFollowingDist set to 20 meters behind the car ahead, which is the location it will try to stabilize around if it approaches too quickly. I use the velocity difference between my vehicle and the car ahead (velDiff) and the distance from the “target” spot are both used. The bigger the difference, the harder the acceleration/deceleration. It tends to work pretty well--though it usually drifts back around to 30m trailing over time if forced to follow for an extended period of time (and can’t shift lanes).
