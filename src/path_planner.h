/*
 * path_planner.h
 *
 * 2D path planner class.
 *  Created on: Dec 28, 2018
 *      Author: Rikki Valverde
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>

// #include "helper_functions.h"
// #include "map.h"
 
// struct Particle {

// 	int id;
// 	double x;
// 	double y;
// 	double theta;
// 	double weight;
// 	std::vector<int> associations;
// 	std::vector<double> sense_x;
// 	std::vector<double> sense_y;
// };

struct ValueArray {
	// std::vector<double> next_x_vals;
	// std::vector<double> next_y_vals;
};

class PathPlanner {
	
	// // Number of particles to draw
	// int num_particles; 
	
	  // vector<double> map_x;
	  // vector<double> map_y;
	  // vector<double> map_s;
	  // vector<double> map_dx;
	  // vector<double> map_dy;	
	
	// Flag, if filter is initialized
	bool is_initialized = 0;
	
	// // Vector of weights of all particles
	// std::vector<double> weights;
	
public:

	// Constructor
	// @param num_particles Number of particles
	PathPlanner() : is_initialized(false) {}

	// Destructor
	~PathPlanner() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x);

	void init(double car_x,double car_y,double car_s, double car_d, double car_yaw, double car_speed);

	// /**
	//  * prediction Predicts the state for the next time step
	//  *   using the process model.
	//  * @param delta_t Time between time step t and t+1 in measurements [s]
	//  * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	//  *   standard deviation of yaw [rad]]
	//  * @param velocity Velocity of car from t to t+1 [m/s]
	//  * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	//  */
	ValueArray planPath(double car_x,double car_y,double car_s, double car_d, double car_yaw, double car_speed, double *previous_path_x, double *previous_path_y, double end_path_s, double end_path_d, double sensor_fusion[][7]);
	
	// /**
	//  * dataAssociation Finds which observations correspond to which landmarks (likely by using
	//  *   a nearest-neighbors data association).
	//  * @param predicted Vector of predicted landmark observations
	//  * @param observations Vector of landmark observations
	//  */
	// void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);
	
	// /**
	//  * updateWeights Updates the weights for each particle based on the likelihood of the 
	//  *   observed measurements. 
	//  * @param sensor_range Range [m] of sensor
	//  * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	//  * @param observations Vector of landmark observations
	//  * @param map Map class containing map landmarks
	//  */
	// void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations,
	// 		const Map &map_landmarks);
	
	// /**
	//  * resample Resamples from the updated set of particles to form
	//  *   the new set of particles.
	//  */
	// void resample();

	// /*
	//  * Set a particles list of associations, along with the associations calculated world x,y coordinates
	//  * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	//  */
	// void SetAssociations(Particle& particle, const std::vector<int>& associations,
	// 	                 const std::vector<double>& sense_x, const std::vector<double>& sense_y);

	
	// std::string getAssociations(Particle best);
	// std::string getSenseX(Particle best);
	// std::string getSenseY(Particle best);

	/**
	* initialized Returns whether particle filter is initialized yet or not.
	*/
	const bool initialized() const {
		return is_initialized;
	}
};



#endif /* PATH_PLANNER_H_ */
