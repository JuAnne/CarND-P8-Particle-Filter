/*
 * particle_filter.cpp
 *
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

// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
// http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
// http://www.cplusplus.com/reference/random/default_random_engine/
// http://en.cppreference.com/w/cpp/types/numeric_limits
// Declare a default random engine that will be used by multiple methods.
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Set the number of particles. Initialize all particles to first position (based on estimates of 
  // x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  num_particles = 100;
  // Create normal (Gaussian) distribution for x, y and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize all particles to the first GPS position (mean value) with normal distribution
  for (int i = 0; i < num_particles; i++)
  {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
  
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Create normal (Gaussian) distribution for sensor noise
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  // Add measurements to each particle and add random Gaussian noise.
  //double epsilon = 10e-5;
  //cout<<"Call ParticleFilter::prediction ---"<< endl;

  for (int i = 0; i < num_particles; i++)
  {
    // when yaw rate is zero, updating x and y.
    if (fabs(yaw_rate) < 0.00001)
    {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    else // yaw rate is not zero, updating x, y and yaw.
    {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
    //add noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
    //cout << "particle id= " << i << " x= "<< particles[i].x <<" y= "<< particles[i].y <<" theta= "<< particles[i].theta << endl;
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // Find the predicted measurement that is closest to each observed measurement and assign the 
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
  //   implement this method and use it as a helper during the updateWeights phase.
  // Using the nearest neighbour techniques , the complexity is O(mn) where m 
  // is the number of map landmarks and n is the number of sensor observations
  //cout <<"Enter ParticleFilter::dataAssociation ---" << endl;
  int m = predicted.size();
  int n = observations.size();
  for (int i = 0; i < n; i++) // for each observation
  {
    double dist_min = numeric_limits<double>::max();
    double dist_curr = 0;// current distance between observation and prediction
    int mapId = -1;
    for(int j = 0; j < m; j++)// for each prediction
    {
      dist_curr = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if(dist_curr < dist_min)
      {
        dist_min = dist_curr;
        mapId = predicted[j].id; // update id association with the nearest neighbour
      }
    }
    observations[i].id = mapId;
  }
  //cout <<"Exit ParticleFilter::dataAssociation ---" << endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation 
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html
  //cout <<"Enter ParticleFilter::updateWeights ---" << endl;
  for ( int i = 0; i < num_particles; i++)   // for each particle
  {
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;
    //cout << "particle id= " << i << " p_x= "<< p_x <<" p_y= "<< p_y <<" p_theta= "<< p_theta << endl;

    // Find landmark predictions that are within sensor range of the current particle, creat a vector to hold them.
    vector<LandmarkObs> lm_pred;
    for( unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      int lm_id = map_landmarks.landmark_list[j].id_i;
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      //cout << "landmark id= " << lm_id << " lm_x= "<< lm_x <<" lm_y= "<< lm_y << endl;

      double distance = dist(lm_x, lm_y, p_x, p_y);
	//cout << "distance= " << distance << endl;
      if( distance < sensor_range )
        lm_pred.push_back(LandmarkObs{lm_id, lm_x, lm_y});
    }
    //int lm_size = lm_pred.size();
    //cout <<"filtered lm size= " << lm_size << endl;

    // Use homogenous transformation equations (perform rotation and translation) to transfer observations
    // from car coordinates to map coordinates
    vector<LandmarkObs>obs_trans;
    for( unsigned int j = 0; j < observations.size(); j++)
    {
      double map_x = p_x + cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y;
	double map_y = p_y + sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y;
	obs_trans.push_back(LandmarkObs{observations[j].id, map_x, map_y});
    }
    //int obs_size = obs_trans.size();
    //cout <<"transformed obs size= " << obs_size << endl;
	
    // Data association for current particle, input sound predictions and transformed observations
    dataAssociation( lm_pred, obs_trans );

    particles[i].weight = 1.0;
    // for each transformed observation
    for(unsigned int j = 0; j < obs_trans.size(); j++)
    {
      double lm_pred_x, lm_pred_y;
	double obs_trans_x = obs_trans[j].x ;
	double obs_trans_y = obs_trans[j].y;
      //find associated landmark by comparing id
      for (unsigned int k = 0; k < lm_pred.size(); k++)
      	{
      	    if (lm_pred[k].id == obs_trans[j].id)
      	    {
      	        lm_pred_x = lm_pred[k].x;
      	        lm_pred_y = lm_pred[k].y;
		  cout<<"found paring id= " << lm_pred[k].id << " k= " << k << " j= " << j << endl;
		  break;
      	    }
      	}

	// calculate weights for this observation using Multivariate-Gaussian Probability
	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1];
      // this observation weight
	double obs_trans_w = (1/(2 * M_PI * sigma_x * sigma_y))
					* exp(-(pow(lm_pred_x - obs_trans_x, 2)/(2*pow(sigma_x, 2))
					         + pow(lm_pred_y - obs_trans_y, 2)/(2*pow(sigma_y, 2))));

      // this particle weight is the product of total observation weights
      particles[i].weight *= obs_trans_w;
    }
    cout<<"particle id= " << i << " weight= " << particles[i].weight <<endl;
  }
  //cout <<"Exit ParticleFilter::updateWeights ---" << endl;
}

void ParticleFilter::resample() {
  //Resample particles with replacement with probability proportional to their weight. 
  // NOTE: You may find std::discrete_distribution helpful here.
  // http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<double>weights;
  vector<Particle>p_resample;
  // calculate max weight
  double max_w = numeric_limits<double>::min();
  for ( int i = 0; i< num_particles; i++)
  {
    weights.push_back(particles[i].weight);
    if(particles[i].weight > max_w)
      max_w = particles[i].weight;
  }

  discrete_distribution<int> dis_index(0, num_particles - 1);
  uniform_real_distribution<double> dis_beta(0.0, 2 * max_w);
  int index = dis_index(gen);
  double beta = 0.0;

  for ( int i = 0; i< num_particles; i++)
  {
    beta += dis_beta(gen);
    while(beta > weights[index])
    {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    p_resample.push_back(particles[index]);
  }

  particles = p_resample;
  
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
