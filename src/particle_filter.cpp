#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <cstring>

#include "particle_filter.h"

using namespace std;

//=============================================================================
//  @brief: ParticleFilter::Init()
//          Initializes particle filter by initializing particles to Gaussian
//          distribution around first position and all the weights to 1.
//
//  @param x Initial x position [m] (simulated estimate from GPS)
//  @param y Initial y position [m]
//  @param theta  Initial orientation [rad]
//  @param std[]  Array of dimension 3 [standard deviation of x [m], 
//                standard deviation of y [m] standard deviation of yaw [rad]]
//  @return: void
//=============================================================================
void ParticleFilter::init(  const double x, const double y, 
                            const double theta, const double std[] ) 
{
  // Initialize x,y,theta with a Gaussion distribution
  std::normal_distribution<double> dist_x( x, std[0] );
  std::normal_distribution<double> dist_y( y, std[1] );
  std::normal_distribution<double> dist_theta( theta, std[2] );

  std::default_random_engine gen;

  // Set the number of particle vector elements equal to NUM_PARTICLES
  num_particles = NUM_PARTICLES;
  particles.resize( num_particles );
  cout << __func__ << ": Particle vector resized to " << num_particles << endl;

  // Iterate over each particle struct in the vector and assign gaussian
  // distribution for x,y,theta
  for( auto& p: particles ) {
    p.x = dist_x( gen );
    p.y = dist_y( gen );
    p.theta = dist_theta( gen );
    p.weight = 1.0;
  }

  is_initialized = true;
  cout << __func__ << ": Particle Filter Initialization Complete" << endl;
}

//=============================================================================
//  @brief: ParticleFilter::prediction()
//          Predicts the state for the next time step using the process model.
//
//  @param delta_t Time between time step t and t+1 in measurements [s]
//  @param std_pos[]  Array of dimension 3 
//                    [standard deviation of x [m], 
//                    standard deviation of y [m], 
//                    standard deviation of yaw [rad]]
//  @param velocity Velocity of car from t to t+1 [m/s]
//  @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
//
//  @return: void
//=============================================================================
void ParticleFilter::prediction( const double delta_t, const double std_pos[], 
                                 const double velocity, const double yaw_rate ) 
{

  // Add Gaussian noise for x,y,theta
  std::normal_distribution<double> N_x( 0, std_pos[0] );
  std::normal_distribution<double> N_y( 0, std_pos[1] );
  std::normal_distribution<double> N_theta( 0, std_pos[2] );

  std::default_random_engine gen;

  // Iterate over each particle struct entry and set prediction for x,y,theta 
  for( auto& p: particles ) {
    if( fabs( yaw_rate ) < 0.0001 ){  
      p.x += velocity * delta_t * cos( p.theta );
      p.y += velocity * delta_t * sin( p.theta );

    } else {
      p.x += velocity / yaw_rate * ( sin( p.theta + yaw_rate*delta_t ) - sin( p.theta ) );
      p.y += velocity / yaw_rate * ( cos( p.theta ) - cos( p.theta + yaw_rate*delta_t ) );
      p.theta += yaw_rate * delta_t;
    }

    // Add noise to the predicted particles
    p.x += N_x( gen );
    p.y += N_y( gen );
    p.theta += N_theta( gen );
  }
}

//=============================================================================
//  @brief: ParticleFilter::getInRangeLandMarks()
//          Get landmarks within the sensor range
//
//  @param sensor_range, 
//  @param reference to particle struct,
//  @param reference to map_landmarks,
//  @param reference to range_landmarks vector
//
//  @return void
//=============================================================================
void ParticleFilter::getInRangeLandmarks( const double sensor_range, 
                                          const Particle& particle,
                                          const Map& map_landmarks,
                                          std::vector<LandmarkObs>& range_landmarks ) 
{
  double min_dist = 0.0;

  // Iterate over map_landmarks and save landmarks within sensor range
  for( const auto& landmark: map_landmarks.landmark_list ) {
    min_dist = dist( particle.x, particle.y, landmark.x_f, landmark.y_f );
    if( min_dist < sensor_range ) {
      range_landmarks.push_back( LandmarkObs{ landmark.id_i, 
                                              landmark.x_f, 
                                              landmark.y_f } );
    }
  }
}

//=============================================================================
//  @brief: ParticleFilter::dataAssociation()
//          Finds which observations correspond to which landmarks 
//          (likely by using a nearest-neighbors data association).
//
//  @param range_landmarks vector 
//  @param map_coords vector 
//
//  @return: void
//=============================================================================
void ParticleFilter::dataAssociation( const std::vector<LandmarkObs> range_landmarks, 
                                      std::vector<LandmarkObs>& map_coords ) 
{
  for( auto& mc: map_coords ) {

      // Initialize minimum distance to maximum number
      auto min_dist = numeric_limits<double>::max();

      // Init map_id to negative number
      int map_id = -1;
      
      // Iterate over range landmarks to find min distance 
      for( const auto& rl: range_landmarks ) {
        // Compute Euclidean distance
        auto cur_dist = dist( mc.x, mc.y, rl.x, rl.y );
  
        // Find the predicted landmark nearest the current observed landmark
        if( cur_dist < min_dist ) {
          min_dist = cur_dist;
          map_id = rl.id;
        }
      }
      // Set map_coords id to predicted id
      mc.id = map_id;
  }
}

//=============================================================================
//  @brief: ParticleFilter::convertVehicleToMap()
//          Convert observations coordinates from vehicle to map
//
//  @param reference to particle struct
//  @param reference to observations vector 
//  @param reference to map_coords vector 
//
//  @return void
//=============================================================================
void ParticleFilter::convertVehicleToMap( const Particle& particle, 
                                          const std::vector<LandmarkObs> &observations, 
                                          std::vector<LandmarkObs>& map_coords ) 
{
  
  LandmarkObs converted;

  for( const auto& obs: observations ) {
    std::memset( &converted, 0, sizeof( converted ) );
    converted.x = ( obs.x*cos( particle.theta ) ) - ( obs.y*sin( particle.theta ) ) + particle.x;
    converted.y = ( obs.x*sin( particle.theta ) ) + ( obs.y*cos( particle.theta ) ) + particle.y;
    map_coords.push_back( converted );
  }
}

//=============================================================================
//  @brief: ParticleFilter::computeWeight()
//          Compute the particle weight
//
//  @param map_coords vector
//  @param map_landmarks
//  @param range_landmarks vector
//  @param std_landmarks
//
//  @return double weight 
//=============================================================================
void ParticleFilter::computeWeight( const std::vector<LandmarkObs> map_coords, 
                                    const Map& map_landmarks,
                                    const std::vector<LandmarkObs> range_landmarks, 
                                    const double std_landmark[],
                                    Particle& particle )
{
  double x = 0.0;
  double y = 0.0;
  double weight = 1.0;

  for( const auto& mc: map_coords ) {
    Map::single_landmark_s landmark = map_landmarks.landmark_list.at( mc.id-1 );
    x = pow( mc.x - landmark.x_f, 2 ) / ( 2 * pow( std_landmark[0], 2 ) );
    y = pow( mc.y - landmark.y_f, 2 ) / ( 2 * pow( std_landmark[1], 2 ) );
    weight = exp( -(x + y) ) / ( 2 * M_PI * std_landmark[0] * std_landmark[1] );
    particle.weight *= weight;
  }
}

//=============================================================================
//  @brief: ParticleFilter::updateWeight()
//          Updates the weights for each particle based on the likelihood of 
//          the observed measurements. 
//
//  @param sensor_range Range [m] of sensor
//  @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty 
//                                              [x [m], 
//                                              y [m]]]
//  @param observations Vector of landmark observations
//  @param map Map class containing map landmarks
//
//  @return: void
//=============================================================================
void ParticleFilter::updateWeights( const double sensor_range, 
                                    const double std_landmark[], 
                                    const std::vector<LandmarkObs> &observations, 
                                    const Map &map_landmarks ) 
{
  
  std::vector<LandmarkObs> range_landmarks;
  std::vector<LandmarkObs> map_coords; 

  for( auto& particle: particles ) {
    particle.weight = 1.0;

    // Get landmarks within sensor_range
    std::memset( &range_landmarks, 0, sizeof( range_landmarks ) );
    getInRangeLandmarks( sensor_range, particle, map_landmarks, range_landmarks );

    // Convert observations coordinates from vehicle to map
    std::memset( &map_coords, 0, sizeof( map_coords ) );
    convertVehicleToMap( particle, observations, map_coords );

    // Find landmark index for each observation
    dataAssociation( range_landmarks, map_coords );

    // Compute the weight 
    computeWeight( map_coords, map_landmarks, range_landmarks, std_landmark, particle );

    weights.push_back( particle.weight );
  }
}

//=============================================================================
//  @brief: ParticleFilter::resample()
//          Set a particles list of associations, along with the associations 
//          calculated world x,y coordinates.
//          This can be a very useful debugging tool to make sure 
//          transformations are correct and assocations correctly connected.
//  @params: void 
//  @return: void
//=============================================================================
void ParticleFilter::resample() 
{
  // Generate distribution according to weights
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> dist( weights.begin(), weights.end() );

  // Create resampled particles
  vector<Particle> resampled_particles;
  resampled_particles.resize( num_particles );

  // Resample the particles according to weights
  for( int i=0; i<num_particles; i++ ) {
    // Compute Euclidean distance
    int idx = dist( gen );
    resampled_particles[i] = particles[idx];
  }

  // Assign the resampled_particles to the previous particles
  particles = resampled_particles;

  // Clear the weight vector for the next round
  weights.clear();
}

//=============================================================================
//  @brief: ParticleFilter::SetAssociations()
//          Set a particles list of associations, along with the associations 
//          calculated world x,y coordinates.
//          This can be a very useful debugging tool to make sure 
//          transformations are correct and assocations correctly connected.
//
//  @params: Particle& particle, vector associations, sense_x, sense_y
//  @return: Particle 
//=============================================================================
Particle ParticleFilter::SetAssociations( Particle& particle, 
                                          const std::vector<int>& associations, 
                                          const std::vector<double>& sense_x, 
                                          const std::vector<double>& sense_y )
{
	// Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

//=============================================================================
//  @brief: ParticleFilter::getAssociations()
//
//  @params: Particle best
//  @return: string 
//=============================================================================
string ParticleFilter::getAssociations( Particle best )
{
	vector<int> v = best.associations;
	stringstream ss;

  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

//=============================================================================
//  @brief: ParticleFilter::getSenseX()
//
//  @params: Particle best
//  @return: string 
//=============================================================================
string ParticleFilter::getSenseX( Particle best )
{
	vector<double> v = best.sense_x;
	stringstream ss;

  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

//=============================================================================
//  @brief: ParticleFilter::getSenseY()
//
//  @params: Particle best
//  @return: string 
//=============================================================================
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;

  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
