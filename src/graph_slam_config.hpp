#ifndef GRAPH_SLAM_CONFIGURATION_HPP
#define GRAPH_SLAM_CONFIGURATION_HPP


namespace graph_slam 
{

/**
 * Configuration for GICP optimizer
 */
struct GICPConfiguration
{
    double max_correspondence_distance;
    unsigned maximum_iterations;
    double transformation_epsilon;
    double euclidean_fitness_epsilon;
    unsigned correspondence_randomness;
    unsigned maximum_optimizer_iterations;
    double rotation_epsilon;
    double point_cloud_density;
    double max_fitness_score;
    double position_sigma;
    double orientation_sigma;
    double max_sensor_distance;
    
    GICPConfiguration() : max_correspondence_distance(2.5),
                          maximum_iterations(50), transformation_epsilon(1e-5),
                          euclidean_fitness_epsilon(1.0), correspondence_randomness(20),
                          maximum_optimizer_iterations(20), rotation_epsilon(2e-3),
                          point_cloud_density(0.2), max_fitness_score(1.0),
                          position_sigma(0.001), orientation_sigma(0.0001), max_sensor_distance(2.0) {};
};

}

#endif