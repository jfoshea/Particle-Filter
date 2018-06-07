#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};


class ParticleFilter {
	
	// Number of particles to draw
	int num_particles; 
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;

  const int NUM_PARTICLES = 100;
	
public:
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param num_particles Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	void init( double x, double y, double theta, double std[] );

	void prediction(  double delta_t, 
                    double std_pos[],
                    double velocity, 
                    double yaw_rate );
	
  void getInRangeLandmarks( const double sensor_range, 
                            const Particle& particle,
                            const Map& map_landmarks,
                            std::vector<LandmarkObs>& range_landmarks ); 

  void convertVehicleToMap( const Particle& particle, 
                            const std::vector<LandmarkObs> &observations, 
                            std::vector<LandmarkObs>& map_coords ); 

  void dataAssociation( std::vector<LandmarkObs> range_landmarks, 
                        std::vector<LandmarkObs>& map_coords ); 

  double computeWeight( const std::vector<LandmarkObs> map_coords, 
                        const Map& map_landmarks,
                        const std::vector<LandmarkObs> range_landmarks, 
                        const double std_landmark[] );

	void updateWeights( double sensor_range, 
                      double std_landmark[], 
                      const std::vector<LandmarkObs> &observations,
                      const Map &map_landmarks );
	
	void resample();

	Particle SetAssociations( Particle& particle, 
                            const std::vector<int>& associations,
                            const std::vector<double>& sense_x, 
                            const std::vector<double>& sense_y );

	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);

	const bool initialized() const 
  {
		return is_initialized;
	}
};

#endif /* PARTICLE_FILTER_H_ */
