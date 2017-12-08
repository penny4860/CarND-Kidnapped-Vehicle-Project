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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	/*
	 * # Args
	 * 		x, y, theta : GPS measurement
	 * 		std[] : GPS measurement uncertatinty
	 */
	// Set the number of particles
	num_particles = 50;
	particles.resize(num_particles);
	weights.resize(num_particles);

	// Initialize all particles to first position
	// This line creates a normal (Gaussian) distribution for x, y, theta.
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	default_random_engine gen;
	for (int i = 0; i < num_particles; i++) {
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;
	}
	is_initialized = true;
#if 0
	cout << "\n\n";
	for (int i = 0; i < 1000; i++)
	{
		cout << "i=" << i << "	x= " << particles[i].x << "	y= " << particles[i].y << "	theta = " << particles[i].theta << "	w= " << particles[i].weight;
		cout << "\n";
	}
#endif
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// cout << "\n	velocity = " << velocity << ", yaw_rate = " << yaw_rate;
	// 1. Add control vector(velocity & yaw_rate) to each particles
	for (int i = 0; i < num_particles; i++) {

		// cout << "\n	x = " << particles[i].x << ", y = " << particles[i].y << ", theta = " << particles[i].theta;
		if (fabs(yaw_rate) < 0.001) {
			particles[i].x = particles[i].x + velocity * cos(particles[i].theta) * delta_t;
			particles[i].y = particles[i].y + velocity * sin(particles[i].theta) * delta_t;
		} else {
			particles[i].x = particles[i].x +
		        velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y = particles[i].y +
		        velocity / yaw_rate * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
		}
		particles[i].theta = particles[i].theta + yaw_rate * delta_t;

		// cout << ", x = " << particles[i].x << ", y = " << particles[i].y << ", theta = " << particles[i].theta;
	}
	// 2. Add random gaussian noise
	normal_distribution<double> dist_noise_x(0, std_pos[0]);
	normal_distribution<double> dist_noise_y(0, std_pos[1]);
	normal_distribution<double> dist_noise_theta(0, std_pos[2]);
	default_random_engine gen;

	for (int i = 0; i < num_particles; i++) {
		double noise_x = dist_noise_x(gen);
		double noise_y = dist_noise_y(gen);
		double noise_theta = dist_noise_theta(gen);

		particles[i].x += noise_x;
		particles[i].y += noise_y;
		particles[i].theta += noise_theta;
#if 0
		cout << "\n	noise x = " << noise_x << ",  noise y = " << noise_y << ",  noise theta = " << noise_theta;
		cout << "\n";
#endif
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	/*
	 * # Args
	 * 		predicted : landmarks
	 * 		observations : sensor input
	 *
	 * # Assign observations id property
	 */
#if 0
	cout << "\n	predicted " << predicted.size();
	for (int i = 0; i < predicted.size(); i++)
	{
		cout << predicted[i].id << "\n";
	}

	cout << "\n observations " << observations.size();
	for (int i = 0; i < observations.size(); i++)
	{
		cout << observations[i].id << "\n";
	}
#endif

	for (unsigned int i = 0; i < observations.size(); i++)
	{
		double min_dist = 50000;
		int min_index = -1;
		for (unsigned int j = 0; j < predicted.size(); j++)
		{
			double dist = sqrt(pow(observations[i].x - predicted[j].x, 2) + pow(observations[i].y - predicted[j].y, 2));
			// cout << "\n	i= " << i << " j= " << j << " dist= " << dist << " min dist= " << min_dist;
			if (min_dist > dist)
			{
				min_dist = dist;
				min_index = j;
			}
		}
		observations[i].id = predicted[min_index].id;
	}
#if 0
	cout << "\n	predicted result \n" << predicted.size();
	for (int i = 0; i < predicted.size(); i++)
	{
		cout << "	" << predicted[i].x << ", " << predicted[i].y << ", " << predicted[i].id << "\n";
	}

	cout << "\n observations result \n" << observations.size();
	for (int i = 0; i < observations.size(); i++)
	{
		cout << "	" << observations[i].x << ", " << observations[i].y << ", " << observations[i].id << "\n";
	}
#endif
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
#if 0
	cout << "sensor_range = "<< sensor_range << "\n";
	cout << "std_landmark = "<< std_landmark[0] << ", "<< std_landmark[1] << "\n";
	cout << "observations\n";
	for (int i = 0; i < observations.size(); i++)
	{
		double x = observations[i].x;
		double y = observations[i].y;
		int id = observations[i].id;
		cout << "		" << x << ", "<< y << ", "<< id <<"\n";
	}
#endif
	cout << "map\n";
	std::vector<LandmarkObs> pred_landmarks;
	for (unsigned int i = 0; i < map_landmarks.landmark_list.size(); i++)
	{
		float x = map_landmarks.landmark_list[i].x_f;
		float y = map_landmarks.landmark_list[i].y_f;
		int id = map_landmarks.landmark_list[i].id_i;
#if 0
		cout << "		" << x << ", "<< y << ", "<< id <<"\n";
#endif
		LandmarkObs obs;
		obs.x = x;
		obs.y = y;
		obs.id = id;
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
			double o_x = observations[j].x;
			double o_y = observations[j].y;

			double pr_x, pr_y;
			for (unsigned int k = 0; k < pred_landmarks.size(); k++) {
        		if (pred_landmarks[k].id == observations[j].id) {
          			pr_x = pred_landmarks[k].x;
          			pr_y = pred_landmarks[k].y;
          			break;
        		}
      		}
      		double obs_w = 1/gauss_norm * exp( - (pow(pr_x-o_x,2)/na + (pow(pr_y-o_y,2)/nb)) );

      		// product of this obersvation weight with total observations weight
      		particles[i].weight *= obs_w;
		}

		weights[i] = particles[i].weight;

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// TODO: Resample particles with replacement with probability proportional to their weight.
	default_random_engine gen;
	vector<Particle> new_particles;

  	// get all of the current weights
  	vector<double> weights;
  	for (int i = 0; i < num_particles; i++) {
    	weights.push_back(particles[i].weight);
  	}
  	discrete_distribution<int> index(weights.begin(), weights.end());
  	for (unsigned j=0; j<num_particles;j++){
  		const int i = index(gen);
  		new_particles.push_back(particles[i]);
  	}
  	particles = new_particles;
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
