/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/*  Initialize the particle using the gps input & its uncertainty.

		# Args
			x, y, theta : GPS measurement
			std : GPS measurement uncertainty
	 */

	// Set the number of particles
	num_particles = 50;
	particles.resize(num_particles);
	weights.resize(num_particles);

	// Initialize particles
	double init_weight = 1.0;
	normal_distribution<double> dist_x(x, std[0]), dist_y(y, std[1]), dist_theta(theta, std[2]);
	default_random_engine gen;
	for (int i = 0; i < num_particles; i++) {
		particles[i] = {i, dist_x(gen), dist_y(gen), dist_theta(gen), init_weight};
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	/*  Predict particles state using control vector (v, yaw_rate)

		# Args
			delta_t : time
			std_pos : GPS measurement uncertainty
			velocity : control vector 1
			yaw_rate : control vector 2
	 */
	// 1. Process next state according to CTRV-model
	for (int i = 0; i < num_particles; i++) {
		// in case of yaw_rate == 0
		if (fabs(yaw_rate) < 0.001) {
			particles[i].x += velocity * cos(particles[i].theta) * delta_t;
			particles[i].y += velocity * sin(particles[i].theta) * delta_t;
		}
		// in case of yaw_rate != 0
		else {
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
		}
		particles[i].theta = particles[i].theta + yaw_rate * delta_t;
	}
	// 2. Add gaussian noise
	normal_distribution<double> dist_x(0, std_pos[0]), dist_y(0, std_pos[1]), dist_theta(0, std_pos[2]);
	default_random_engine gen;
	for (int i = 0; i < num_particles; i++) {
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	/*  Find the nearest predicted landmark id and assign it to the observation.

		# Args
			predicted (input): predefined landmarks
			observations (input, output): sensor input transformed to map coordinate
	 */
	for (unsigned int i = 0; i < observations.size(); i++)
	{
		double min_dist = 50000;
		int min_index = -1;
		for (unsigned int j = 0; j < predicted.size(); j++)
		{
			double dist = sqrt(pow(observations[i].x - predicted[j].x, 2) + pow(observations[i].y - predicted[j].y, 2));
			if (min_dist > dist)
			{
				min_dist = dist;
				min_index = j;
			}
		}
		if (min_index != -1)
			observations[i].id = predicted[min_index].id;
		else
			cout << "\n warning : dataAssociation Error";
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	/* Calculate likelihood and update weights (particles[i].weight, weight)

		# Args
			sensor_range :
			std_landmark :
				Sensor (e.g. lidar) measurement uncertainty.
			observations :
				Sensor measurements in vehicle coordinate system.
				This information is used as "measurements" in the Baysian rule.
			map_landmarks :
				predefined landmark position in map coordinate system
				This information is used as "prior" in the Baysian rule.
	 */

	// 1. Get predicted landmarks
	std::vector<LandmarkObs> pred_landmarks;
	for (unsigned int i = 0; i < map_landmarks.landmark_list.size(); i++)
	{
		LandmarkObs obs;
		obs.x = map_landmarks.landmark_list[i].x_f;
		obs.y = map_landmarks.landmark_list[i].y_f;
		obs.id = map_landmarks.landmark_list[i].id_i;
		pred_landmarks.push_back(obs);
	}

	for (int i = 0; i < num_particles; i++)
	{
		double xp = particles[i].x;
		double yp = particles[i].y;
		double theta_p = particles[i].theta;
		std::vector<LandmarkObs> meas_landmarks;

		// 1. particle coordinate to map coordinate
		for (unsigned int j = 0; j < observations.size(); j++)
		{
			double xc = observations[j].x;
			double yc = observations[j].y;

			double xm = xp + cos(theta_p)*xc - sin(theta_p)*yc;
			double ym = yp + sin(theta_p)*xc + cos(theta_p)*yc;

			LandmarkObs obs;
			obs.x = xm;
			obs.y = ym;
			obs.id = observations[j].id; //??
			meas_landmarks.push_back(obs);
		}
		// 2. matching nearest landmarks
		dataAssociation(pred_landmarks, meas_landmarks);

		// 3. update weights
		particles[i].weight = 1.0;

		double std_x = std_landmark[0];
		double std_y = std_landmark[1];
		double na = 2.0 * std_x * std_x;
		double nb = 2.0 * std_y * std_y;
		double gauss_norm = 2.0 * M_PI * std_x * std_y;

		for (unsigned j=0; j < observations.size(); j++){
			double o_x = meas_landmarks[j].x;
			double o_y = meas_landmarks[j].y;

			double pr_x, pr_y;
			for (unsigned int k = 0; k < pred_landmarks.size(); k++) {
        		if (pred_landmarks[k].id == meas_landmarks[j].id) {
          			pr_x = pred_landmarks[k].x;
          			pr_y = pred_landmarks[k].y;
          			break;
        		}
      		}
      		double obs_w = 1/gauss_norm * exp( - (pow(pr_x-o_x,2)/na + (pow(pr_y-o_y,2)/nb)) );
      		particles[i].weight *= obs_w;
		}
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	/* Resampling the particles according to the weight. */

	default_random_engine gen;
	vector<Particle> resampled_particles;
  	vector<double> weights;
  	weights.resize(num_particles);
  	resampled_particles.resize(num_particles);

  	// 1. Get particles weights
  	for (int i = 0; i < num_particles; i++) {
    	weights[i] = particles[i].weight;
  	}

  	// 2. Sample index of particle
  	discrete_distribution<int> index(weights.begin(), weights.end());
  	for (unsigned j=0; j<num_particles;j++){
  		int i = index(gen);
  		resampled_particles[i] = particles[i];
  	}
  	particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
