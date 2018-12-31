#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "path_planner.h"

using namespace std;

void PathPlanner::init(double car_x,double car_y,double car_s, double car_d, double car_yaw, double car_speed) {


    is_initialized = 1;

    ValueArray vf;
    // return vf;

}


ValueArray PathPlanner::planPath(double car_x,double car_y,double car_s, double car_d, double car_yaw, double car_speed, double *previous_path_x, double *previous_path_y, double end_path_s, double end_path_d, double sensor_fusion[][7]) {

    // starting from end path

    // convert position to s and d coordinates
    // pick a point 10m ahead
    // plan a jerk/acceleration approved path to get there - find coordinates
    // convert to time based using dt

    // check for nearest car ahead
    // if slower than v_des, slow down
    // if really slow, plan a lane change
    // look for a break in traffic to the left
    // pass the car and move back to right
    // if stopped, don't hit it

    for (int i=0; i<12; i++){
        std::cout << sensor_fusion[i][5] << std::endl;
    }

    // see what sensor fusion returns

    ValueArray vf;
    return vf;

}