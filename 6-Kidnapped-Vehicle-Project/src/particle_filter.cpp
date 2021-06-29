/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std;

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 10;  // TODO: Set the number of particles
  //std::default_random_engine gen;
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  // initialization
  for(int i=0; i<num_particles; i++){
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    
    // append the particle to the whole particles vector
    particles.push_back(p);    
  }
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],   //std_pos:x, y, theta measurement uncertainities
                                double velocity, double yaw_rate) {   // veloc, Yaw of current time step
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.  //sampling gaussian dist. with mean equal to updated particle pos,   std dev= std dev of the mesurement
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  for (int i=0; i<num_particles; i++){
    double x_0 = particles[i].x;
    double y_0 = particles[i].y;
    double theta = particles[i].theta;
    
    double x_f=0, y_f=0, theta_f=0;
    
    // if yaw_rate!=0
    if(fabs(yaw_rate)>0.00001){
      double yaw_d = yaw_rate*delta_t;
      x_f = x_0 + (velocity/yaw_rate) * (sin(theta+yaw_d) - sin(theta));
      y_f = y_0 + (velocity/yaw_rate) * (cos(theta) - cos(theta + yaw_d));
      theta_f = theta + yaw_d; 
    }
    
    // if yaw_rate=0
    else{ 
      x_f = x_0 + velocity * delta_t *  cos(theta);
      y_f = y_0 + velocity * delta_t *  sin(theta);
      theta_f = theta;
    }
    
    //get the distribution of the predicted with noise
    normal_distribution<double> dist_x(x_f, std_pos[0]);
    normal_distribution<double> dist_y(y_f, std_pos[1]);
    normal_distribution<double> dist_theta(theta_f, std_pos[2]);
    
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  //cout<<"id: "<<observations.size()<<'\n';
  for(int i=0; i<observations.size(); i++){
    double min_dist = 50;    //alternative, use this:  double min_dist = numeric_limits<double>::max();
    double distance=0;
    
    for(int j=0; j<predicted.size(); j++){
      distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      
      if(distance<min_dist){
        observations[i].id = predicted[j].id;
        //cout<<"id: "<<observations[j].id<<"    j: "<<j<<'\n';
        min_dist = distance;
      }
    }
  }
}

// to calculate the weight
double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))+ (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
  return weight;
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];
  double normalizer = 0.0;
    
  //calculate the weights for each particle    
  for (int i=0; i<num_particles; i++){
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    
    // get landmarks in the sensor range
    vector<LandmarkObs> predicted_landmarks;
    for(int m=0; m<map_landmarks.landmark_list.size(); ++m){
      int id_ =map_landmarks.landmark_list[m].id_i;
      double x_ =map_landmarks.landmark_list[m].x_f;
      double y_ =map_landmarks.landmark_list[m].y_f;
      double distance = dist(x, y, x_, y_);
      if(distance<sensor_range){
        LandmarkObs predicted_l;
        predicted_l.id = id_;
        predicted_l.x = x_;
        predicted_l.y = y_;
        predicted_landmarks.push_back(predicted_l);
      }
    }
    
    // transform the observation to the map coordination
    vector<LandmarkObs> observations_trans;
    for(int j=0; j<observations.size(); j++){      
      LandmarkObs obs_trans;
      double x_obs = observations[j].x;
      double y_obs = observations[j].y;
      obs_trans.x = x + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      obs_trans.y = y + (sin(theta) * x_obs) + (cos(theta) * y_obs);
      observations_trans.push_back(obs_trans);
    }
      
    // Associate the data 
    dataAssociation(predicted_landmarks, observations_trans);
    double w =1.0;
    double mu_x;
    double mu_y;
    
    // get the accumlated weight for each particle
    for (int z=0; z<observations_trans.size(); z++){
      // get mu needed for calculating the weights
      for (const auto& land: predicted_landmarks)
        if (observations_trans[z].id == land.id) {
          mu_x = land.x;
          mu_y = land.y;
          break;
        }

      w *= multiv_prob(sig_x, sig_y, observations_trans[z].x, observations_trans[z].y, mu_x, mu_y);
      
    }
    //update the weight
    particles[i].weight = w;
    //get the normalizer
    normalizer += particles[i].weight;
  } // end for particles

  // weights normalization
  for (int n=0; n<particles.size(); n++){
    particles[n].weight /= (normalizer + numeric_limits<double>::epsilon());
  }   
}
    

    
void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  //random distribution Initialization
  vector<double> weights;
    for(int in=0; in<particles.size(); in++){
      weights.push_back(particles[in].weight);
    }
        
  discrete_distribution<int> distribution(weights.begin(), weights.end());
  //get the resampled particles
  vector<Particle> p_resampled;
  p_resampled.resize(num_particles);
  for(int i = 0; i<num_particles; i++){
    p_resampled[i] = particles[distribution(gen)];
  }

  //update particles
  particles = p_resampled; 
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}