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

static default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  num_particles = 5;


  normal_distribution<double> dist_x( x, std[0]);
  normal_distribution<double> dist_y( y, std[1]);
  normal_distribution<double> dist_theta( theta, std[2]);
  //normal_distribution<double> dist_x( 0, 1);
  //normal_distribution<double> dist_y( 0, 1);
  //normal_distribution<double> dist_theta( 0.0, 0.03);

  //weights.reserve(num_particles); //saves runtime
  for (int i=0; i<num_particles; i++){
   // weights[i]=1;
    Particle p = {};

    // assign values to p
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    // add p to your particles vector
    particles.push_back(p);
    //cout<<p.x<<endl;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/


  int xf, yf, thetaf; // so as not to overwrite
  for (int i = 0; i < num_particles; i++) {
    if (yaw_rate != 0) {
      xf = particles[i].x +
           velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      yf = particles[i].y +
           velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      thetaf = particles[i].theta + yaw_rate * delta_t;

    } else {
      xf = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      yf = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      thetaf = particles[i].theta;
    }
    // adding random noise with error std
    normal_distribution<double> pos_error_x(particles[i].x, std_pos[0]);
    normal_distribution<double> pos_error_y(particles[i].y, std_pos[1]);
    normal_distribution<double> pos_error_theta(particles[i].theta, std_pos[2]);
    particles[i].x = pos_error_x(gen);
    particles[i].y = pos_error_y(gen);
    particles[i].theta = pos_error_theta(gen);

  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  //   implement this method and use it as a helper during the updateWeights phase.



  std::vector<double> distances;
  double min_dist;
  int predicted_min_id;

  double distance;


  for (int j = 0; j < observations.size(); ++j) {
    min_dist = 9999.0;

    for (int k = 0; k < predicted.size(); ++k) {
      distance = dist(observations[j].x, observations[j].y, predicted[k].x, predicted[k].y);
      if (distance < min_dist) {
        min_dist = distance;
        predicted_min_id = predicted[k].id;

      }
    }

    observations[j].id = predicted_min_id;

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




  LandmarkObs trnsfrmd;

  double car_obs_x;
  double car_obs_y;
  double particle_x;
  double particle_y;
  double particle_heading;

  for(int i = 0; i < particles.size(); ++i) {
    particle_x = particles[i].x;
    particle_y = particles[i].y;
    particle_heading = particles[i].theta;

    printf(" particle x %.2f  particle y %.2f  particle theta %.2f endl", particles[i].x, particles[i].y,
           particles[i].theta);
    std::vector<LandmarkObs> transformed_observations;
    // Observations transformations
    for (int j = 0; j < observations.size(); ++j) {
      car_obs_x = observations[j].x;
      car_obs_y = observations[j].y;

      trnsfrmd.x = particle_x + car_obs_x * cos(particle_heading) - car_obs_y * sin(particle_heading);
      trnsfrmd.y = particle_y + car_obs_x * sin(particle_heading) + car_obs_y * cos(particle_heading);
      trnsfrmd.id = observations[j].id;

      transformed_observations.push_back(trnsfrmd);

    }
    //cout<<"transformed_observations size = "<<transformed_observations.size()<<endl;
    // Up until here, we have a vector of the transformed observations.


    // Landmark predictions

    std::vector<LandmarkObs> in_range_predicted_obs;
    LandmarkObs single_in_range_pred_obs, nearest_landmark;
    double range;
    double landmark_map_x;
    double landmark_map_y;
    double landmark_map_id;
    float min_distance = 200;

    for (int k = 0; k < map_landmarks.landmark_list.size(); ++k) {
      float landmark_map_x = map_landmarks.landmark_list[k].x_f;
      float landmark_map_y = map_landmarks.landmark_list[k].y_f;
      int landmark_map_id = map_landmarks.landmark_list[k].id_i;
      int nearest_landmark_id;
      // calculate distance between particle and landmark
      range = dist(landmark_map_x, landmark_map_y, particle_x, particle_y);

      if (range <= sensor_range * sensor_range)// landmark in sensor range of particle
      {
        if (range < min_distance) {
          min_distance = range;
          nearest_landmark.x = landmark_map_x;
          nearest_landmark.y = landmark_map_y;
        }

      }

    }





    // Association step
    //SetAssociations(in_range_predicted_obs, transformed_observations);


    // Weight calculation step

    double std_landmark_x = std_landmark[0];
    double std_landmark_y = std_landmark[1];
    double weight = 1.0;



    double x_val = nearest_landmark.x;
    double y_val = nearest_landmark.y;

    double normalizer = 1 / (2 * M_PI * std_landmark_x * std_landmark_y);
    double exponent = exp(-(((pow((x_val - particle_x), 2)) / (2 * std_landmark_x * std_landmark_x)) +
                            ((pow((y_val - particle_y), 2)) / (2 * std_landmark_y * std_landmark_y))));
    printf("normalizer = %.5f  exponent = %.6f \n", normalizer, exponent);
    weight *= normalizer * exponent;


    cout << "weight = " << weight << endl;

  }


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution




  vector<double> pdist;
  double total = 0;
  for (int i = 0; i < num_particles; i++)
  {
    total += particles[i].weight;
    pdist.push_back(particles[i].weight);
  }

  for (int i = 0; i < num_particles; i++) pdist[i] /= total;
  // Create New distribution
  std::discrete_distribution<int> ddist(pdist.begin(), pdist.end());

  // Resample according to the weights
  vector<Particle> resampled_particles(num_particles);
  for (int i = 0; i < num_particles; i++)
  {
    resampled_particles[i] = particles[ddist(gen)];
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

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

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
