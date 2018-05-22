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
	if (is_initialized) {
		
		return;

	}

	num_particles = 100;

	default_random_engine gen;
	Particle part;


	// Create a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std[0]);

	// Create normal distributions for y and theta
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);


	for (int i = 0; i < num_particles; i++) {
		

		//Sample  and from these normal distrubtions 
		
		part.id = i;
		part.x = dist_x(gen);
		part.y = dist_y(gen);
		part.theta = dist_theta(gen);

		part.weight = 1.0;

		particles.push_back(part);
		
		weights.push_back(1.0);
		
	}
	is_initialized = true;
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;



	// Create a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(0, std_pos[0]);

	// Create normal distributions for y and theta
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);


	for (int i = 0; i < num_particles; i++) {


		//predict new position: movement + noise 
		
		// if yaw rate is too small
		if (fabs(yaw_rate) < 1E-6) { 

			particles[i].x = particles[i].x + velocity * delta_t * cos(particles[i].theta) + dist_x(gen);

			particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta) + +dist_y(gen);


		}
		else {
			particles[i].x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) + dist_x(gen);
			particles[i].y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) + dist_y(gen);
			particles[i].theta = particles[i].theta + yaw_rate * delta_t + dist_theta(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	 

	double min_distance;
	double delta;

	

    vector<bool> found(predicted.size(), false);

	for (int i = 0; i < observations.size(); i++) {

		

		// find closest predicted measurement

		min_distance = 1E10;

		for (int j = 0; j < predicted.size(); j++) {

			

			delta = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			if (delta < min_distance) {

				min_distance = delta;
				
				observations[i].id = j;


			}

		}

		

	}
	
	
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

	double EPSW = 1E-10; // minimum weight
	vector<int> associations;

	vector<double> sense_x, sense_y;

	for (int i = 0; i < num_particles; i++) {

		

		// create list of landmarks that are approximately within sensor_range of the particle

		vector<LandmarkObs> near_landmarks;

		for (int j = 0; j < map_landmarks.landmark_list.size(); j++)

			if (abs(particles[i].x - map_landmarks.landmark_list[j].x_f) <= sensor_range &&

				abs(particles[i].y - map_landmarks.landmark_list[j].y_f) <= sensor_range)

				near_landmarks.push_back(LandmarkObs{ map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });

	
		vector<LandmarkObs> obs_maps;

		LandmarkObs obs_map;
				       
		// map observations to world coordinate system


		for (int j = 0; j < observations.size(); j++) {

			obs_map.x = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;

			obs_map.y = particles[i].y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;

			obs_maps.push_back(obs_map);

		}


		// associate observations with the landmarks

		dataAssociation(near_landmarks, obs_maps);

		
		// update weights 

		for (int j = 0; j < obs_maps.size(); j++) {

			if (obs_maps[j].id != -1) {

				// observation is associated with some landmark, update the weight of the particle

				double deltax = obs_maps[j].x - near_landmarks[obs_maps[j].id].x;
				double deltay = obs_maps[j].y - near_landmarks[obs_maps[j].id].y;

				//cout << dx << endl; 

				double new_weight;

				double weight_exp = (deltax*deltax / (2 * std_landmark[0] * std_landmark[0])) + (deltay*deltay / (2 * std_landmark[1] * std_landmark[1]));

				// use multivariate Gaussian distribution
				new_weight = (1 / (2 * M_PI*std_landmark[0] * std_landmark[1])) * exp(-weight_exp);
				
								
				//cout << weight_exp;
				//cout << new_weight<<endl;

				
				particles[i].weight = particles[i].weight*new_weight;
				


				// update associations data

				associations.push_back(near_landmarks[obs_maps[j].id].id);

				sense_x.push_back(obs_maps[j].x);

				sense_y.push_back(obs_maps[j].y);

			}

		}



		// avoid that all particles are zero 

		if (particles[i].weight < EPSW)

			particles[i].weight = EPSW;



		// update weights

		weights[i] = particles[i].weight;

		
		// set associations 

		SetAssociations(particles[i], associations, sense_x, sense_y);

	}

	// normalize weights
	double sum_weights = 0.0;
	for (int i = 0; i < num_particles; i++) {
		sum_weights += weights[i];
	}
	for (int i = 0; i < num_particles; i++) {
		particles[i].weight = particles[i].weight/sum_weights;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	// Get weights and max weight.
	
	default_random_engine gen;

	
	// find maximum weight
	double max_weight = 0.0;

	for (int i = 0; i < num_particles; i++) {

		if (weights[i] > max_weight) {
			max_weight = particles[i].weight;
		}

	}
	
	
	// needed distributions
	uniform_real_distribution<double> dist_real(0.0, 2*max_weight);

	uniform_int_distribution<int> dist_int(0, num_particles - 1);
		
	int indx = dist_int(gen); // index of sampled particle

	double beta = 0.0;

	
	// the random wheel

	vector<Particle> new_particles;

	for (int i = 0; i < num_particles; i++) {

		beta = beta + dist_real(gen);

		while (weights[indx] < beta) {

			beta = beta - weights[indx];

			indx = (indx + 1) % num_particles;

		}

		new_particles.push_back(particles[indx]);

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

	return particle;
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
