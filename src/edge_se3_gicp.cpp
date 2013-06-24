#include "edge_se3_gicp.hpp"

namespace graph_slam
{
    
EdgeSE3_GICP::EdgeSE3_GICP() : EdgeSE3(), run_gicp(true), use_guess_from_state(false)
{
    setGICPConfiguration(GICPConfiguration());
    
    _measurement = Eigen::Isometry3d::Identity();
    _inverseMeasurement = Eigen::Isometry3d::Identity();
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
}

bool EdgeSE3_GICP::setMeasurementFromGICP()
{
    if(icp.getInputTarget().get() == NULL || icp.getInputCloud().get() == NULL)
        return false;
    
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    if(use_guess_from_state)
    {
        Eigen::Matrix4f guess = Eigen::Isometry3f(_inverseMeasurement).matrix();
        icp.align(cloud_source_registered, guess);
    }
    else
    {
        icp.align(cloud_source_registered);
    }
    
    Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
    transformation.matrix() = icp.getFinalTransformation();
    _inverseMeasurement = Eigen::Isometry3d(transformation);
    _measurement = _inverseMeasurement.inverse();
    run_gicp = false;
    return true;
}

bool EdgeSE3_GICP::setMeasurementFromOdometry()
{
    graph_slam::VertexSE3_GICP *from = static_cast<graph_slam::VertexSE3_GICP*>(_vertices[0]);
    graph_slam::VertexSE3_GICP *to = static_cast<graph_slam::VertexSE3_GICP*>(_vertices[1]);
    
    _measurement = from->getOdometryPose().inverse() * to->getOdometryPose();
    _inverseMeasurement = _measurement.inverse();
    _information = from->getOdometryCovariance().inverse() * to->getOdometryCovariance();
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

void EdgeSE3_GICP::runGICP()
{
    run_gicp = true;
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