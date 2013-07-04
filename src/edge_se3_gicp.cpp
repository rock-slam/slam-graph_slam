#include "edge_se3_gicp.hpp"
#include <graph_slam/matrix_helper.hpp>

namespace graph_slam
{
    
EdgeSE3_GICP::EdgeSE3_GICP() : EdgeSE3(), run_gicp(true), use_guess_from_state(false), max_fitness_score(1.0), 
                               position_sigma(0.001), orientation_sigma(0.0001), valid_gicp_measurement(false)
{
    setGICPConfiguration(GICPConfiguration());
    
    _measurement = Eigen::Isometry3d::Identity();
    _inverseMeasurement = Eigen::Isometry3d::Identity();
    _information = Matrix6d::Zero();
}

void EdgeSE3_GICP::setGICPConfiguration(const GICPConfiguration& gicp_config)
{
    icp.setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp.setMaximumIterations(gicp_config.maximum_iterations);
    icp.setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(gicp_config.euclidean_fitness_epsilon);
    icp.setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp.setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp.setRotationEpsilon(gicp_config.rotation_epsilon);
    max_fitness_score = gicp_config.max_fitness_score;
    position_sigma = gicp_config.position_sigma;
    orientation_sigma = gicp_config.orientation_sigma;
}

bool EdgeSE3_GICP::setMeasurementFromGICP(bool delayed)
{
    if(delayed)
    {
        run_gicp = true;
        return true;
    }
    
    if(icp.getInputTarget().get() == NULL || icp.getInputCloud().get() == NULL)
        return false;
    
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    if(use_guess_from_state)
    {
        /* NOTE gicp seems to be broken when a inital guess is given
        if(_inverseMeasurement.matrix() == Eigen::Isometry3d::Identity().matrix())
            setMeasurementFromOdometry();
        Eigen::Matrix4f guess = Eigen::Isometry3f(_inverseMeasurement).matrix();
        icp.align(cloud_source_registered, guess);
        */
        icp.align(cloud_source_registered, Eigen::Matrix4f::Identity());
    }
    else
    {
        icp.align(cloud_source_registered);
    }
    
    double fitness_score = icp.getFitnessScore();
    if(fitness_score <= max_fitness_score)
    {
        Eigen::Isometry3f transformation(icp.getFinalTransformation());
        _inverseMeasurement = Eigen::Isometry3d(transformation);
        _measurement = _inverseMeasurement.inverse();

        _information = combineToPoseCovariance(pow(position_sigma,2) * Eigen::Matrix3d::Identity(), pow(orientation_sigma,2) * Eigen::Matrix3d::Identity()).inverse();
        
        valid_gicp_measurement = true;
    }
    
    run_gicp = false;
    return true;
}

void EdgeSE3_GICP::computeError()
{
    if(run_gicp)
        setMeasurementFromGICP();
    EdgeSE3::computeError();
}

void EdgeSE3_GICP::initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to)
{
    if(run_gicp)
        setMeasurementFromGICP();
    EdgeSE3::initialEstimate(from, to);
}

void EdgeSE3_GICP::linearizeOplus()
{
    if(run_gicp)
        setMeasurementFromGICP();
    EdgeSE3::linearizeOplus();
}

void EdgeSE3_GICP::setSourceVertex(VertexSE3_GICP* source)
{
    _vertices[0]=source;
    icp.setInputCloud(source->getPCLPointCloud());
}

void EdgeSE3_GICP::setTargetVertex(VertexSE3_GICP* target)
{
    _vertices[1]=target;
    icp.setInputTarget(target->getPCLPointCloud());
}

}