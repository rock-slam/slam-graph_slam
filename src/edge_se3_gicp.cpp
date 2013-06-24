#include "edge_se3_gicp.hpp"

namespace graph_slam
{
    
EdgeSE3_GICP::EdgeSE3_GICP() : EdgeSE3(), run_gicp(true)
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

bool EdgeSE3_GICP::setMeasurementFromState()
{
    if(icp.getInputTarget().get() == NULL || icp.getInputCloud().get() == NULL)
        return false;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    // Perform the alignment
    icp.align(cloud_source_registered);
    Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
    transformation.matrix() = icp.getFinalTransformation();
    _measurement = Eigen::Isometry3d(transformation);
    _inverseMeasurement = _measurement.inverse();
    return true;
}

void EdgeSE3_GICP::computeError()
{
    if(run_gicp)
        setMeasurementFromState();
    EdgeSE3::computeError();
}

void EdgeSE3_GICP::initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to)
{
    if(run_gicp)
        setMeasurementFromState();
    EdgeSE3::initialEstimate(from, to);
}

void EdgeSE3_GICP::linearizeOplus()
{
    if(run_gicp)
        setMeasurementFromState();
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