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
	default_random_engine gen;
	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	num_particles = 10;

	for(int i = 0; i < num_particles; i++) {
		Particle particle;

		// This line creates a normal (Gaussian) distribution for x, y, and theta
		normal_distribution<double> dist_x(x, std_x);
		normal_distribution<double> dist_y(y, std_y);
		normal_distribution<double> dist_theta(theta, std_theta);

		// Sample and from these normal distributions
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);

		// Set initial weight to 1
		particle.weight = 1.0;
		weights.push_back(1.0);

		particles.push_back(particle);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];
	//cout << "in predication" << endl;
	for(int i = 0; i < num_particles; i++) {
		Particle &particle = particles[i];
		//cout << "before prediction, particle " << i << " " << particle.x <<" " << particle.y <<" " << particle.theta <<" " << endl;
		double pred_theta;
		pred_theta = particle.theta + yaw_rate * delta_t;
		if(yaw_rate == 0) {
			particle.x += velocity * delta_t * sin(particle.theta);
			particle.y += velocity * delta_t * cos(particle.theta);
		}
		else {
			particle.x += (velocity / yaw_rate) * (sin(pred_theta) -  sin(particle.theta));
			particle.y += (velocity / yaw_rate) * (cos(particle.theta) - cos(pred_theta));
		}
		particle.theta = pred_theta;

		// This line creates a normal (Gaussian) distribution for x, y, and theta
		normal_distribution<double> dist_x(particle.x, std_x);
		normal_distribution<double> dist_y(particle.y, std_y);
		normal_distribution<double> dist_theta(particle.theta, std_theta);

		// Sample and from these normal distributions
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		//cout << "after prediction, particle " << i << " " << particle.x <<" " << particle.y <<" " << particle.theta <<" " << endl;
		//cout << endl;
	}

}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//To Reviewer: I didn't implement this function as I did all the word in function of updateWeights()

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


	// cout << "in update" << endl;
    // cout << "the size of observations is: " << observations.size() << endl;
	float std_lm_x = std_landmark[0];
	float std_lm_y = std_landmark[1];
	float gauss_norm = 1/(2*M_PI*std_lm_x*std_lm_y);

	for(int i = 0; i < num_particles; i++) {
		Particle &particle = particles[i];
		// cout <<"updating particle: " <<" " <<  particle.x <<" " <<  particle.y <<" " <<  particle.weight << endl;
		float particle_x, particle_y, particle_theta;
		particle_x = particle.x;
		particle_y = particle.y;
		particle_theta = particle.theta;

		float prob = 1.0;
		for(int j = 0; j < observations.size(); j++) {
			// cout << observations[j].x <<" " << observations[j].y << endl;

			// Homogenous Transformation: from Car coordinates to Map coordinates
			float obs_x, obs_y, tobs_x, tobs_y;
			obs_x = observations[j].x;
			obs_y = observations[j].y;
			tobs_x = particle_x + cos(particle_theta) * obs_x - sin(particle_theta) * obs_y;
			tobs_y = particle_y + sin(particle_theta) * obs_x +  cos(particle_theta) * obs_y;
			// cout << "obs_x, tobs_x, obs_y, tobs_y: " << obs_x << " "  << tobs_x << " " <<  obs_y  << " " << tobs_y << endl;

			// Find the closest map landmark for each observation/measurement
			float shortest_dist = INT_MAX;
			int closest_landmark_x, closest_landmark_y;
			for(int k = 0; k < map_landmarks.landmark_list.size(); k++) {
				float landmark_x, landmark_y, tobs_landmark_dist;
				landmark_x = map_landmarks.landmark_list[k].x_f;
				landmark_y = map_landmarks.landmark_list[k].y_f;
				tobs_landmark_dist = dist(tobs_x, tobs_y, landmark_x, landmark_y);
				if(shortest_dist > tobs_landmark_dist) {
					shortest_dist =  tobs_landmark_dist;
					closest_landmark_x = landmark_x;
					closest_landmark_y = landmark_y;
					}
				}
			// cout << "find closet landmark: " << tobs_x <<" " << closest_landmark_x <<" " << tobs_y <<" " << closest_landmark_y << endl;

			// Calculate particle weights and final weight
			float exponent = pow(tobs_x - closest_landmark_x, 2) / (2 * pow(std_lm_x, 2))
							 + pow(tobs_y - closest_landmark_y, 2) / (2 * pow(std_lm_y, 2));

			prob *= gauss_norm *  exp(-exponent);


		}

		// Update weights of particles
		particle.weight = prob;
		weights[i] = prob;
		//cout <<"update particle done: " <<" " <<  particle.x <<" " <<  particle.y <<" " <<  particle.weight << endl;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Resample particles with replacement with probability proportional to their weight
	std::vector<Particle> particles_resampled;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());
    for(int n=0; n<num_particles; ++n) {
        int i = d(gen);
        Particle p;
        p.x = particles[i].x;
        p.y = particles[i].y;
        p.theta = particles[i].theta;
        p.weight = 1.0;
        particles_resampled.push_back(p);
    }

    // Update particles and weights
    for(int n=0; n<num_particles; ++n) {
    		particles[n] = particles_resampled[n];
    		weights[n] = 1.0;
    }


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
